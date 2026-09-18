// 악보를 서버와 같은 경로(BehaviorPlanner → PlayMotionGenerator)로 하드웨어 없이 재생한다.
//  - log/offline_<ID>_trajectory.csv : 서버 trajectory 로그와 같은 열 (t, joint 0..12), t 는 5 ms 간격. 첫 행 = reset 자세
//  - BaseMotionGenerator 를 같은 입력으로 병행 실행해 위치 수준 검사: 타격점이 후보인지, 같은 악기 위 R.x > L.x,
//    같은 악기 반복 시 후보 유지, 창 경계 위치 점프, 타격 시 양팔 허리 폭
// 빌드/실행: drumrobot_server/test/play_score_check.sh <ID>... (cwd = 저장소 루트)
#include "trajectory/behavior_planner.hpp"
#include "trajectory/play_motion_generator.hpp"
#include "trajectory/base_motion_generator.hpp"

#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

static constexpr int N_WAIST = 1801;
static double waist_sample_angle(int i) { return -0.5 * M_PI + M_PI / 1800.0 * i; }
static int physical_id(int inst) { return inst == 9 ? 5 : inst; }
static int note_of(const DrumEvent& e, bool right) {
    int note = right ? e.note_num_R : e.note_num_L;
    if (note == 5 && !e.is_closed_hihat) note = 9;
    return note;
}
static double dist(const std::array<double, 3>& a, const std::array<double, 3>& b) {
    return std::sqrt((a[0]-b[0])*(a[0]-b[0]) + (a[1]-b[1])*(a[1]-b[1]) + (a[2]-b[2])*(a[2]-b[2]));
}

// drum_coordinate.json (통합 파일) → PlayMotionGenerator::initialize 와 같은 좌표/후보 (BaseMotionGenerator 직접 구동용)
static std::map<int, InstrumentCoordinate> load_coordinates() {
    std::map<int, InstrumentCoordinate> coords;
    nlohmann::json root;
    std::ifstream("drumrobot_server/config/drum_coordinate.json") >> root;
    for (const auto& inst : root.at("instruments")) {
        InstrumentCoordinate c;
        const auto& ctr = inst.at("center");
        c.center = {ctr.at(0).get<double>(), ctr.at(1).get<double>(), ctr.at(2).get<double>()};
        c.wrist_angle = inst.at("wrist_angle_deg").get<double>() * M_PI / 180.0;
        std::vector<std::array<double, 3>> candidates;
        for (const auto& p : inst.value("candidates", nlohmann::json::array())) {
            candidates.push_back({p.at(0).get<double>(), p.at(1).get<double>(), p.at(2).get<double>()});
        }
        if (candidates.empty()) candidates.push_back(c.center);
        for (auto p : candidates) {
            p[0] += ROBOT::CANDIDATE_HAND_X_OFFSET;        c.right_candidate_positions.push_back(p);
            p[0] -= 2.0 * ROBOT::CANDIDATE_HAND_X_OFFSET;  c.left_candidate_positions.push_back(p);
        }
        coords[instrument_name_to_id.at(inst.at("name").get<std::string>())] = c;
    }
    return coords;
}

int main(int argc, char** argv) {
    if (argc < 2) { std::cerr << "usage: play_score_check <play_list ID>...\n"; return 1; }
    const auto coords = load_coordinates();
    KinematicsSolver solver; solver.initialize();

    int total_issues = 0;
    for (int a = 1; a < argc; a++) {
        const std::string id = argv[a];
        std::cout << "\n===== " << id << " =====\n";
        std::cerr << "===== " << id << " =====\n";     // stderr(선택 로그)도 곡별로 나눌 수 있게

        AppContext ctx; Robot robot; AudioPlayer audio;      // 하드웨어 초기화 없음
        BehaviorPlanner planner(ctx, robot, audio);
        ctx.send_active = true;                             // PLAY 는 send_active 이후·IDLE 에서만 수락됨 (behavior_planner.cpp:48)
        ctx.robot_state = RobotState::IDLE;
        ParsedCommand cmd; cmd.valid = true; cmd.opcode = Opcode::PLAY; cmd.args = {id};
        std::vector<MotionPrimitive> sequence = planner.generate_motion_sequence(cmd);
        if (sequence.size() < 3) { std::cout << "[issue] 시퀀스 생성 실패 (" << sequence.size() << " primitives)\n"; total_issues++; continue; }

        PlayMotionGenerator pmg(ctx); pmg.initialize();     // 실제 서버 경로
        BaseMotionGenerator bmg;      bmg.initialize(coords); // 병행 검사용 (같은 입력 → 같은 출력이어야 함)

        std::ofstream csv("drumrobot_server/log/offline_" + id + "_trajectory.csv");
        csv << "t"; for (int j = 0; j < ROBOT::NUM_JOINT; j++) csv << ",joint " << j; csv << "\n";
        csv << std::fixed << std::setprecision(4);
        long row = 0;
        auto write_row = [&](const std::array<double, ROBOT::NUM_JOINT>& q) {
            csv << row * ROBOT::DT_SECOND; for (double v : q) csv << "," << v; csv << "\n"; row++;
        };

        int round_sum = 0;   // PlayMotionGenerator::get_num_point 복제 (speed 1.0)
        auto num_point = [&](double t0, double t1) {
            double n = (t1 - t0) / ROBOT::DT_SECOND;
            round_sum += (int)(n * 10000) % 10000;
            if (round_sum >= 10000) { round_sum -= 10000; n++; }
            return (int)std::floor(n);
        };

        // 검사 상태
        int issues = 0, windows = 0, hits = 0, waist_mismatch = 0;
        int repeat_moved[2] = {0, 0}, repeat_total[2] = {0, 0};
        int last_phys[2] = {0, 0}, last_idx[2] = {-1, -1};
        size_t min_width = N_WAIST; double min_width_t = 0.0; int zero_width = 0;
        bool have_prev = false; BaseMotionPoint prev{};
        double max_step[2] = {0.0, 0.0};

        for (const auto& motion : sequence) {
            if (motion.type != MotionType::DRUM) continue;
            if (motion.flag == PlayFlag::START) {
                std::array<double, ROBOT::NUM_JOINT> q{};
                if (!pmg.reset(q, motion.init_note_r, motion.init_note_l)) { std::cout << "[issue] reset IK 실패\n"; issues++; break; }
                bmg.reset(motion.init_note_r, motion.init_note_l);
                write_row(q);
                continue;
            }
            if (motion.flag != PlayFlag::PLAYING) continue;
            const auto& rds = motion.robotic_drum_score;
            windows++;

            const int n = num_point(rds[0].t, rds[1].t);
            std::queue<std::array<double, ROBOT::NUM_JOINT>> q_queue = pmg.generate_motion(rds);
            std::queue<BaseMotionPoint> b_queue = bmg.generate_motion(rds, n, ROBOT::DT_SECOND);
            if (q_queue.empty()) {
                std::cout << "[issue] t=" << rds[0].t << " PlayMotionGenerator 가 빈 큐 반환 (IK 실패 또는 base 오류) → 서버라면 연주 중단\n";
                // 진단: 병행 실행 창의 점마다 계획 허리각 vs 양팔 동시 가능 허리 구간
                int k = 0, out = 0;
                while (!b_queue.empty()) {
                    const BaseMotionPoint b = b_queue.front(); b_queue.pop();
                    std::bitset<N_WAIST> f;
                    for (int i = 0; i < N_WAIST; i++) {
                        const double th = waist_sample_angle(i);
                        f[i] = solver.check_joint_limit(0, th)
                            && solver.solve_arm_ik(b.right_position, th, b.right_wrist, KinematicsSolver::ArmSide::RIGHT, false).success
                            && solver.solve_arm_ik(b.left_position,  th, b.left_wrist,  KinematicsSolver::ArmSide::LEFT,  false).success;
                    }
                    int lo = -1, hi = -1;
                    for (int i = 0; i < N_WAIST; i++) if (f[i]) { if (lo < 0) lo = i; hi = i; }
                    const double planned = b.waist * 180.0 / M_PI;
                    const bool bad = (lo < 0) || planned < waist_sample_angle(lo) * 180.0 / M_PI || planned > waist_sample_angle(hi) * 180.0 / M_PI;
                    if (bad) out++;
                    if (k == 0 || (bad && out <= 8)) {
                        std::cout << "        k=" << k << " t=" << rds[0].t + k * ROBOT::DT_SECOND << " 계획 허리 " << planned << "deg, 가능 구간 ";
                        if (lo < 0) std::cout << "없음"; else std::cout << waist_sample_angle(lo) * 180.0 / M_PI << " ~ " << waist_sample_angle(hi) * 180.0 / M_PI << "deg";
                        std::cout << " R=[" << b.right_position[0] << "," << b.right_position[1] << "," << b.right_position[2] << "]"
                                  << " L=[" << b.left_position[0] << "," << b.left_position[1] << "," << b.left_position[2] << "]" << (bad ? "  <-- 밖" : "") << "\n";
                    }
                    k++;
                }
                std::cout << "        창 " << k << "점 중 구간 밖 " << out << "점\n";
                issues++; break;
            }
            if (q_queue.size() != b_queue.size()) {
                std::cout << "[issue] t=" << rds[0].t << " 병행 실행 점 개수 불일치 " << q_queue.size() << " vs " << b_queue.size() << "\n";
                issues++; break;
            }

            bool first = true;
            while (!q_queue.empty()) {
                const auto q = q_queue.front(); q_queue.pop();
                const BaseMotionPoint b = b_queue.front(); b_queue.pop();
                write_row(q);
                if (std::abs(q[0] - b.waist) > 1e-6) waist_mismatch++;

                if (have_prev) {
                    const double dR = dist(prev.right_position, b.right_position), dL = dist(prev.left_position, b.left_position);
                    max_step[0] = std::max(max_step[0], dR); max_step[1] = std::max(max_step[1], dL);
                    if (first && (dR > 0.03 || dL > 0.03)) {   // 창 경계에서 한 스텝(5 ms)에 3 cm 초과 = 위치 불연속
                        std::cout << "[issue] t=" << rds[0].t << " 창 경계 위치 점프 R=" << dR * 100 << "cm L=" << dL * 100 << "cm\n"; issues++;
                    }
                }
                prev = b; have_prev = true;

                if (first) {
                    first = false;
                    // 타격 순간(창 첫 점 = rds[0].t): 후보 소속 / 좌우 순서 / 반복 유지 / 허리 폭
                    const int nR = note_of(rds[0], true), nL = note_of(rds[0], false);
                    const std::array<double, 3> pos[2] = {b.right_position, b.left_position};
                    const int note[2] = {nR, nL};
                    for (int h = 0; h < 2; h++) {
                        if (note[h] == 0) { continue; }
                        hits++;
                        const auto& c = coords.at(note[h]);
                        const auto& cands = h == 0 ? c.right_candidate_positions : c.left_candidate_positions;
                        const bool in_cands = std::find(cands.begin(), cands.end(), pos[h]) != cands.end();
                        if (!in_cands) { std::cout << "[issue] t=" << rds[0].t << " " << (h ? "L" : "R") << " 악기 " << note[h] << " 타격점이 후보가 아님\n"; issues++; }
                        // 같은 물리 악기 반복: 후보 인덱스가 유지되는지 (closed/open hihat 은 z 만 달라 인덱스로 비교)
                        const int idx = in_cands ? (int)(std::find(cands.begin(), cands.end(), pos[h]) - cands.begin()) : -1;
                        if (last_phys[h] == physical_id(note[h])) {
                            repeat_total[h]++;
                            if (idx != last_idx[h]) {
                                repeat_moved[h]++;
                                if (repeat_moved[h] <= 6) std::cout << "[info]  t=" << rds[0].t << " " << (h ? "L" : "R") << " 악기 " << note[h]
                                                                    << " 반복인데 후보 " << last_idx[h] << " -> " << idx << " (동시 상대 악기 " << note[1 - h] << ")\n";
                            }
                        }
                        last_phys[h] = physical_id(note[h]); last_idx[h] = idx;
                    }
                    if (nR && nL && physical_id(nR) == physical_id(nL) && !(b.right_position[0] > b.left_position[0])) {
                        std::cout << "[issue] t=" << rds[0].t << " 같은 악기 " << nR << " 위 좌우 순서 위반 R.x=" << b.right_position[0] << " L.x=" << b.left_position[0] << "\n"; issues++;
                    }
                    if (nR || nL) {
                        std::bitset<N_WAIST> fR, fL;
                        for (int i = 0; i < N_WAIST; i++) {
                            const double th = waist_sample_angle(i);
                            if (!solver.check_joint_limit(0, th)) continue;
                            fR[i] = solver.solve_arm_ik(b.right_position, th, b.right_wrist, KinematicsSolver::ArmSide::RIGHT, false).success;
                            fL[i] = solver.solve_arm_ik(b.left_position,  th, b.left_wrist,  KinematicsSolver::ArmSide::LEFT,  false).success;
                        }
                        const size_t w = (fR & fL).count();
                        if (w == 0) zero_width++;
                        if (w < min_width) { min_width = w; min_width_t = rds[0].t; }
                    }
                }
            }
        }

        std::cout << "windows=" << windows << " hits=" << hits << " rows=" << row
                  << " | 타격 시 최소 허리 폭 " << min_width * 0.1 << "deg (t=" << min_width_t << "), 폭 0 타격 " << zero_width
                  << " | 같은 악기 반복 중 후보 변경 R " << repeat_moved[0] << "/" << repeat_total[0] << ", L " << repeat_moved[1] << "/" << repeat_total[1]
                  << " | 최대 스텝 R " << max_step[0] * 100 << "cm L " << max_step[1] * 100 << "cm"
                  << " | 병행 실행 허리 불일치 " << waist_mismatch << "\n";
        std::cout << (issues ? "ISSUES: " : "clean: ") << issues << "\n";
        total_issues += issues;
    }
    return total_issues ? 2 : 0;
}
