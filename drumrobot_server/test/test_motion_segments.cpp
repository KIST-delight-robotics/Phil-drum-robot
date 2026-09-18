// 양손 동시 후보점 선정 확인. 하드웨어 없이 BaseMotionGenerator 공개 API 만 사용한다.
// 빌드/실행 (cwd = 저장소 루트, kinematics.json 이 상대경로). 한 줄 명령:
//   g++ -std=c++17 -O2 -Wall -Idrumrobot_server/include -Idrumrobot_server/lib drumrobot_server/test/test_motion_segments.cpp
//       drumrobot_server/src/trajectory/base_motion_generator.cpp drumrobot_server/src/kinematics/kinematics_solver.cpp
//       -o /tmp/test_motion_segments && /tmp/test_motion_segments
#include "trajectory/base_motion_generator.hpp"

#include <cassert>
#include <chrono>
#include <fstream>

static constexpr int N = 1801;
static double waist_sample_angle(int i) { return -0.5 * M_PI + M_PI / 1800.0 * i; }

// 테스트 독립 스윕: 생성기의 캐시를 쓰지 않고 KinematicsSolver 로 직접 구한다
static std::bitset<N> sweep(const KinematicsSolver& solver, const std::array<double, 3>& p, double wrist, KinematicsSolver::ArmSide side) {
    std::bitset<N> f;
    for (int i = 0; i < N; i++) {
        const double t0 = waist_sample_angle(i);
        f[i] = solver.check_joint_limit(0, t0) && solver.solve_arm_ik(p, t0, wrist, side, false).success;
    }
    return f;
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

static DrumEvent ev(double t, int r = 0, int l = 0) { DrumEvent e; e.t = t; e.note_num_R = r; e.note_num_L = l; e.is_closed_hihat = true; return e; }
static bool contains(const std::vector<std::array<double, 3>>& v, const std::array<double, 3>& p) { return std::find(v.begin(), v.end(), p) != v.end(); }
static BaseMotionPoint last(std::queue<BaseMotionPoint> q) { while (q.size() > 1) q.pop(); return q.front(); }

int main() {
    const auto coords = load_coordinates();   // 후보는 0828_1139 스캔 (drum_coordinate.json 에 통합)
    const auto& cand_R6 = coords.at(6).right_candidate_positions;   // ride, 후보 3개
    const auto& cand_L2 = coords.at(2).left_candidate_positions;    // floor, 후보 9개
    assert(cand_R6.size() >= 2 && cand_L2.size() >= 2);

    BaseMotionGenerator gen;
    gen.initialize(coords);

    // ===== 1) 양손 동시 새 비행 (스네어 휴식 → t=0.6 에 R=ride, L=floor) → 쌍 탐색 =====
    gen.reset(1, 1);
    auto q1 = gen.generate_motion({ev(0.0), ev(0.6, 6, 2), ev(1.2)}, 6, 0.1);
    auto q2 = gen.generate_motion({ev(0.6, 6, 2), ev(1.2)}, 6, 0.1);    // 타격 순간: 첫 점 = 선정된 도착점
    assert(!gen.get_error());
    const BaseMotionPoint hit = q2.front();
    assert(contains(cand_R6, hit.right_position));
    assert(contains(cand_L2, hit.left_position));

    // 독립 브루트포스: 동시 타격·다른 악기라 보간·순서 제약 없음 → max_{i,j} |feasR(i) & feasL(j)|
    KinematicsSolver solver; solver.initialize();
    const double wR = coords.at(6).wrist_angle, wL = coords.at(2).wrist_angle;
    std::vector<std::bitset<N>> fR, fL;
    for (const auto& p : cand_R6) fR.push_back(sweep(solver, p, wR, KinematicsSolver::ArmSide::RIGHT));
    for (const auto& p : cand_L2) fL.push_back(sweep(solver, p, wL, KinematicsSolver::ArmSide::LEFT));
    size_t best = 0;
    for (const auto& a : fR) for (const auto& b : fL) best = std::max(best, (a & b).count());
    const size_t iR = std::find(cand_R6.begin(), cand_R6.end(), hit.right_position) - cand_R6.begin();
    const size_t jL = std::find(cand_L2.begin(), cand_L2.end(), hit.left_position) - cand_L2.begin();
    const size_t chosen = (fR[iR] & fL[jL]).count();
    std::cout << "pair: R cand " << iR << "/" << cand_R6.size() << ", L cand " << jL << "/" << cand_L2.size()
              << ", width " << chosen * 0.1 << " deg (brute-force max " << best * 0.1 << " deg)\n";
    assert(chosen == best && best > 0);

    // ===== 2) 비행 중 잠금: 두 스텝에 걸친 비행에서 도착점이 바뀌지 않는다 =====
    gen.reset(1, 1);
    auto s1 = last(gen.generate_motion({ev(0.0), ev(0.3), ev(0.6, 6, 2), ev(1.2)}, 3, 0.1));   // 비행 시작 (선정)
    auto s2 = last(gen.generate_motion({ev(0.3), ev(0.6, 6, 2), ev(1.2)}, 3, 0.1));            // 비행 지속 (재선정 없음)
    auto s3 = gen.generate_motion({ev(0.6, 6, 2), ev(1.2)}, 3, 0.1).front();                   // 도착
    (void)s1; (void)s2;
    assert(!gen.get_error());
    assert(s3.right_position == hit.right_position && s3.left_position == hit.left_position);

    // ===== 3) 비동시 타격 (R t=0.6, L t=0.9): 보간점 스윕 N+M 회 경로. 소요 시간만 출력 =====
    gen.reset(1, 1);
    const auto t_begin = std::chrono::steady_clock::now();
    auto q3 = gen.generate_motion({ev(0.0), ev(0.6, 6, 0), ev(0.9, 0, 2), ev(1.5)}, 6, 0.1);
    const auto ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t_begin).count();
    auto q4 = gen.generate_motion({ev(0.6, 6, 0), ev(0.9, 0, 2), ev(1.5)}, 3, 0.1);
    assert(!gen.get_error());
    assert(contains(cand_R6, q4.front().right_position));
    std::cout << "non-simultaneous pair select: generate_motion took " << ms << " ms (incl. waist lookahead)\n";

    std::cout << "OK\n";
    return 0;
}
