#include "trajectory/base_motion_generator.hpp"

BaseMotionGenerator::BaseMotionGenerator() {

}

BaseMotionGenerator::~BaseMotionGenerator() {

}

void BaseMotionGenerator::initialize() {
    using json = nlohmann::json;
 
    solver.initialize();
 
    // 기본값 (파일 로드 실패 시 fallback) : PathManager 의 손목각 기본값과 동일
    for (int i = 0; i < ROBOT::NUM_INSTRUMENT; i++) {
        drum_position_R[i] = {0.0, 0.0, 0.0};
        drum_position_L[i] = {0.0, 0.0, 0.0};
        wrist_angle_on_impact_R[i] = 10.0 * M_PI / 180.0;
        wrist_angle_on_impact_L[i] = 10.0 * M_PI / 180.0;
    }
 
    std::ifstream f("drumrobot/config/drum_coordinate.json");
    if (!f.is_open()) {
        std::cerr << "[BaseMotionGenerator] Failed to open config/drum_coordinate.json\n";
        return;
    }
 
    json config = json::parse(f, nullptr, false);
    if (config.is_discarded() || !config.contains("instruments")) {
        std::cerr << "[BaseMotionGenerator] Invalid drum_coordinate.json\n";
        return;
    }
 
    for (auto& inst : config["instruments"]) {
        int id = inst["id"].get<int>();     // 1~10
        if (id < 1 || id > ROBOT::NUM_INSTRUMENT) continue;
        int idx = id - 1;
 
        auto load_side = [](const json& side,
                            std::array<double, 3>& pos,
                            double& wrist) {
            auto p = side["position"];
            pos = { p[0].get<double>(), p[1].get<double>(), p[2].get<double>() };
            wrist = side["wrist_angle_deg"].get<double>() * M_PI / 180.0;
        };
 
        if (inst.contains("right")) load_side(inst["right"], drum_position_R[idx], wrist_angle_on_impact_R[idx]);
        if (inst.contains("left"))  load_side(inst["left"],  drum_position_L[idx], wrist_angle_on_impact_L[idx]);
    }
 
    table_loaded = true;
    std::cout << "[BaseMotionGenerator] Drum coordinates loaded.\n";
}
 
// =============================================================
// checkOpenHihat : HH(5) 이고 open 이면 OHH(9) 로 치환
// =============================================================
int BaseMotionGenerator::check_open_hihat(int inst_num, bool is_closed_hihat) const {
    if (inst_num == 5) {
        return is_closed_hihat ? 5 : 9;   // closed -> 5(HH), open -> 9(OHH)
    }
    return inst_num;
}
 
// =============================================================
// getTargetPosition : 악기 번호 -> 좌표 + 손목각
// =============================================================
void BaseMotionGenerator::get_target_position(int inst_num, bool is_right,
                                              std::array<double, 3>& position_out,
                                              double& wrist_angle_out) const {
    if (inst_num < 1 || inst_num > ROBOT::NUM_INSTRUMENT) {
        // 악기 없음/오류 -> 직전 도착점 유지 의도. 0 좌표 반환(호출부에서 처리)
        position_out = {0.0, 0.0, 0.0};
        wrist_angle_out = 0.0;
        return;
    }
    int idx = inst_num - 1;
    if (is_right) {
        position_out = drum_position_R[idx];
        wrist_angle_out = wrist_angle_on_impact_R[idx];
    } else {
        position_out = drum_position_L[idx];
        wrist_angle_out = wrist_angle_on_impact_L[idx];
    }
}
 
// =============================================================
// parseTrajectoryData (한 손) 이식
//   rds 윈도우에서 다음 타격을 lookahead 하여 state/시작·목표 악기 판정
// =============================================================
BaseMotionGenerator::ParsedHand
BaseMotionGenerator::parse_hand(const std::vector<DrumEvent>& rds,
                                bool is_right,
                                const std::array<double, 4>& prev_state) {
    ParsedHand out;
 
    const double e = 0.00001;
    const double hit_detection_threshold = 1.2 * 100.0 / BPM;
 
    // 현재 줄(0) 과 다음 줄들의 시간/악기/offset 접근 헬퍼
    auto inst_of = [&](int i) -> int {
        return is_right ? rds[i].note_num_R : rds[i].note_num_L;
    };
    auto vel_of = [&](int i) -> int {
        // offset 정보가 DrumEvent 에 없으므로 velocity 를 offset 대용으로 쓰지 않음.
        // 원본 offset 컬럼은 새 악보 포맷에 아직 매핑되지 않아 0 으로 둔다.
        (void)i;
        return 0;
    };
    auto time_of = [&](int i) -> double { return rds[i].t; };
    auto closed_of = [&](int i) -> bool { return rds[i].is_closed_hihat; };
 
    const double t0 = time_of(0);
 
    // ----- 타격 감지 (lookahead) -----
    bool detect_hit = false;
    double detect_time = (rds.size() > 1) ? time_of(1) : t0;
    int detect_inst = 0;
    int detect_offset = 0;
 
    for (std::size_t i = 1; i < rds.size(); i++) {
        if (std::round(10000 * (hit_detection_threshold + e)) <
            std::round(10000 * (time_of(i) - t0))) {
            break;
        }
        if (inst_of(i) != 0) {
            detect_hit = true;
            detect_time = time_of(i);
            detect_inst = check_open_hihat(inst_of(i), closed_of(i));
            detect_offset = vel_of(i);
            break;
        }
    }
 
    const double prev_initial_t = prev_state[0];
    const int prev_initial_inst = static_cast<int>(prev_state[1]);
    const int prev_state_val = static_cast<int>(prev_state[2]);
    const int prev_offset = static_cast<int>(prev_state[3]);
 
    int initial_inst, final_inst, next_state;
    double initial_t, final_t;
    int initial_offset, final_offset;
    int is_making = 0;
 
    if (inst_of(0) == 0) {
        // 현재 줄이 타격으로 끝나지 않음
        if (prev_state_val == 2 || prev_state_val == 3) {
            // 이전 줄에서 시작된 궤적 진행 중
            next_state = prev_state_val;
            initial_inst = prev_initial_inst;
            final_inst = detect_inst;
            initial_t = prev_initial_t;
            final_t = detect_time;
            initial_offset = prev_offset;
            final_offset = detect_offset;
            is_making = 1;
        } else {
            if (detect_hit) {
                next_state = 2;
                initial_inst = prev_initial_inst;
                final_inst = detect_inst;
                initial_t = t0;
                final_t = detect_time;
                initial_offset = 0;
                final_offset = detect_offset;
            } else {
                next_state = 0;
                initial_inst = prev_initial_inst;
                final_inst = prev_initial_inst;
                initial_t = t0;
                final_t = (rds.size() > 1) ? time_of(1) : t0;
                initial_offset = 0;
                final_offset = 0;
                is_making = 2;
            }
        }
    } else {
        // 현재 줄이 타격으로 끝남
        int cur_inst = check_open_hihat(inst_of(0), closed_of(0));
        if (detect_hit) {
            next_state = 3;
            initial_inst = cur_inst;
            final_inst = detect_inst;
            initial_t = t0;
            final_t = detect_time;
            initial_offset = vel_of(0);
            final_offset = detect_offset;
        } else {
            next_state = 1;
            initial_inst = cur_inst;
            final_inst = cur_inst;
            initial_t = t0;
            final_t = (rds.size() > 1) ? time_of(1) : t0;
            initial_offset = vel_of(0);
            final_offset = 0;
            is_making = 2;
        }
    }
 
    out.initial_time = initial_t;
    out.final_time = final_t;
    out.initial_inst = initial_inst;
    out.final_inst = final_inst;
    out.initial_offset = initial_offset;
    out.final_offset = final_offset;
    out.is_making_trajectory = is_making;
    out.next_state = { initial_t,
                       static_cast<double>(initial_inst),
                       static_cast<double>(next_state),
                       static_cast<double>(initial_offset) };
    return out;
}
 
// =============================================================
// calTimeScaling : 3차 다항식 (s(ti)=0, s(tf)=1, 양끝 속도 0)
// =============================================================
double BaseMotionGenerator::cal_time_scaling(double ti, double tf, double t) const {
    if (tf <= ti) return 1.0;
    double tau = (t - ti) / (tf - ti);
    if (tau <= 0.0) return 0.0;
    if (tau >= 1.0) return 1.0;
    return 3.0 * tau * tau - 2.0 * tau * tau * tau;
}
 
// =============================================================
// makeTaskSpacePath : 3차 Bezier (PathManager 와 동일)
// =============================================================
std::array<double, 3>
BaseMotionGenerator::make_task_space_path(const std::array<double, 3>& Pi,
                                          const std::array<double, 3>& Pf,
                                          int initial_offset,
                                          int final_offset,
                                          double s) const {
    double dzi = initial_offset * 0.01;
    double dzf = final_offset * 0.01;
    double zi = Pi[2] - dzi;
    double zf = Pf[2] - dzf;
 
    std::array<double, 3> Ps{0.0, 0.0, 0.0};
 
    bool same = (Pi[0] == Pf[0] && Pi[1] == Pf[1] && Pi[2] == Pf[2]);
    if (same) {
        Ps[0] = Pi[0];
        Ps[1] = Pi[1];
        Ps[2] = zi + s * (zf - zi);
        return Ps;
    }
 
    const double h2_min = 0.08;
    double h1 = 2.0 * dzi;
    double h2 = h2_min + 0.6 * std::abs(zi - zf);
 
    std::array<double, 3> Pi_off = Pi, Pf_off = Pf;
    Pi_off[2] = zi;
    Pf_off[2] = zf;
 
    std::array<double, 3> Pm1, Pm2;
    for (int k = 0; k < 3; k++) {
        Pm1[k] = (2.0 / 3.0) * Pi_off[k] + (1.0 / 3.0) * Pf_off[k];
        Pm2[k] = (1.0 / 3.0) * Pi_off[k] + (2.0 / 3.0) * Pf_off[k];
    }
    Pm1[2] = Pi[2] + h1;
    Pm2[2] = Pf[2] + h2;
 
    double u = 1.0 - s;
    for (int k = 0; k < 3; k++) {
        Ps[k] = u * u * u * Pi_off[k]
              + 3.0 * u * u * s * Pm1[k]
              + 3.0 * u * s * s * Pm2[k]
              + s * s * s * Pf_off[k];
    }
    return Ps;
}
 
// =============================================================
// getWaistParams : the0 (-90~90deg) 1801 스캔, IK 해 존재 범위에서
//   비용 최소 허리각 선택
//   cost = W0*cos(theta1+theta2) + W1*cos(w*|theta0 - min|)
// =============================================================
double BaseMotionGenerator::get_waist_params(const std::array<double, 3>& pR,
                                             const std::array<double, 3>& pL,
                                             double theta7, double theta8,
                                             bool& solved_out) const {
    std::vector<std::array<double, 3>> sols;   // [theta0, theta1, theta2]
    sols.reserve(1801);
 
    for (int i = 0; i < 1801; i++) {
        double theta0 = -0.5 * M_PI + M_PI / 1800.0 * i;
        KinematicsSolver::IKResult r = solver.ik_solve(pR, pL, theta0, theta7, theta8);
        if (r.success) {
            sols.push_back({ r.q[0], r.q[1], r.q[2] });
        }
    }
 
    if (sols.empty()) {
        solved_out = false;
        return 0.0;
    }
    solved_out = true;
 
    const double W0 = 2.0, W1 = 1.0;
    double min_q0 = sols.front()[0];
    double max_q0 = sols.back()[0];
 
    double w = (std::abs(max_q0 - min_q0) > 1e-9)
             ? 2.0 * M_PI / std::abs(max_q0 - min_q0)
             : 0.0;
 
    double min_cost = W0 + W1;   // W.sum()
    int min_index = 0;
    for (std::size_t i = 0; i < sols.size(); i++) {
        double theta0 = sols[i][0];
        double theta1 = sols[i][1];
        double theta2 = sols[i][2];
        double cost = W0 * std::cos(theta1 + theta2)
                    + W1 * std::cos(w * std::abs(theta0 - min_q0));
        if (cost < min_cost) {
            min_cost = cost;
            min_index = static_cast<int>(i);
        }
    }
    return sols[min_index][0];   // optimized_q0
}
 
// =============================================================
// generate_motion : rds[0] -> rds[1] 구간의 base 궤적 n개 생성
// =============================================================
std::queue<BaseMotionPoint>
BaseMotionGenerator::generate_motion(std::vector<DrumEvent> rds, int num_point) {
    std::queue<BaseMotionPoint> out;
 
    if (rds.size() < 2 || num_point <= 0) {
        return out;
    }
 
    // ----- 양손 파싱 (state 업데이트) -----
    ParsedHand R = parse_hand(rds, true,  measure_state_R);
    ParsedHand L = parse_hand(rds, false, measure_state_L);
    measure_state_R = R.next_state;
    measure_state_L = L.next_state;
 
    // ----- 악기 좌표/손목각 -----
    std::array<double, 3> init_target_R, init_target_L, final_target_R, final_target_L;
    double init_wrist_R, init_wrist_L, final_wrist_R, final_wrist_L;
    get_target_position(R.initial_inst, true,  init_target_R, init_wrist_R);
    get_target_position(L.initial_inst, false, init_target_L, init_wrist_L);
    get_target_position(R.final_inst,   true,  final_target_R, final_wrist_R);
    get_target_position(L.final_inst,   false, final_target_L, final_wrist_L);
 
    // ----- 첫 호출 시드 -----
    if (!has_last_hit) {
        last_final_position_R = init_target_R;
        last_final_position_L = init_target_L;
        last_initial_position_R = init_target_R;
        last_initial_position_L = init_target_L;
        has_last_hit = true;
    }
 
    // ----- 시작/목표 위치 결정 (is_making_trajectory) -----
    std::array<double, 3> initial_pos_R, final_pos_R, initial_pos_L, final_pos_L;
 
    if (R.is_making_trajectory == 1) {          // 진행 중
        initial_pos_R = last_initial_position_R;
        final_pos_R   = last_final_position_R;
    } else if (R.is_making_trajectory == 2) {   // 제자리 대기
        initial_pos_R = last_final_position_R;
        final_pos_R   = last_final_position_R;
        last_initial_position_R = initial_pos_R;
    } else {                                    // 신규 (0)
        initial_pos_R = last_final_position_R;
        final_pos_R   = final_target_R;         // 충돌회피 없이 악기 좌표 그대로
        last_initial_position_R = initial_pos_R;
    }
 
    if (L.is_making_trajectory == 1) {
        initial_pos_L = last_initial_position_L;
        final_pos_L   = last_final_position_L;
    } else if (L.is_making_trajectory == 2) {
        initial_pos_L = last_final_position_L;
        final_pos_L   = last_final_position_L;
        last_initial_position_L = initial_pos_L;
    } else {
        initial_pos_L = last_final_position_L;
        final_pos_L   = final_target_L;
        last_initial_position_L = initial_pos_L;
    }
 
    // 신규 궤적이면 도착점을 다음 구간 시작점으로 저장
    if (R.is_making_trajectory == 0) last_final_position_R = final_pos_R;
    if (L.is_making_trajectory == 0) last_final_position_L = final_pos_L;
 
    // ----- 손목각 (시작/목표 보간용) -----
    double initial_wrist_R = (R.initial_inst >= 1) ? init_wrist_R : final_wrist_R;
    double initial_wrist_L = (L.initial_inst >= 1) ? init_wrist_L : final_wrist_L;
    double final_wrist_angle_R = final_wrist_R;
    double final_wrist_angle_L = final_wrist_L;
 
    // ----- 구간 시간 -----
    double t1 = rds[0].t;   // 현재 줄 누적시간 (play_motion_generator 의 rds[0].t 와 동일)
    // (PathManager 는 measureMatrix(0,10), measureMatrix(1,10) 사용.
    //  여기서는 parse 결과의 initial/final time 으로 time scaling)
 
    // ----- 허리각 : 구간 시작점(s=0) 기준 1회 최적화 -----
    //   PathManager 는 genTaskSpaceTrajectory 의 i==0 에서 getWaistParams 를
    //   호출하고, 이후 makeWaistCoefficient 로 박들 사이를 보간한다.
    //   여기서는 단순화하여 구간 시작 위치 기준 최적 허리각을 구간 전체에 사용.
    {
        std::array<double, 3> posR0 = make_task_space_path(initial_pos_R, final_pos_R,
                                                          R.initial_offset, R.final_offset, 0.0);
        std::array<double, 3> posL0 = make_task_space_path(initial_pos_L, final_pos_L,
                                                          L.initial_offset, L.final_offset, 0.0);
        bool solved = false;
        double opt = get_waist_params(posR0, posL0, initial_wrist_R, initial_wrist_L, solved);
        if (solved) {
            last_waist_angle = opt;
        }
        // 해 없으면 직전 구간의 허리각(last_waist_angle) 유지
    }
 
    // ----- n개 포인트 생성 -----
    for (int i = 0; i < num_point; i++) {
        BaseMotionPoint p;
 
        double tR = ROBOT::DT_SECOND * i + t1 - R.initial_time;
        double tL = ROBOT::DT_SECOND * i + t1 - L.initial_time;
 
        double sR = cal_time_scaling(0.0, R.final_time - R.initial_time, tR);
        double sL = cal_time_scaling(0.0, L.final_time - L.initial_time, tL);
 
        std::array<double, 3> posR = make_task_space_path(initial_pos_R, final_pos_R,
                                                          R.initial_offset, R.final_offset, sR);
        std::array<double, 3> posL = make_task_space_path(initial_pos_L, final_pos_L,
                                                          L.initial_offset, L.final_offset, sL);
 
        double wristR = sR * (final_wrist_angle_R - initial_wrist_R) + initial_wrist_R;
        double wristL = sL * (final_wrist_angle_L - initial_wrist_L) + initial_wrist_L;
 
        p.right_x = posR[0];
        p.right_y = posR[1];
        p.right_z = posR[2];
        p.left_x  = posL[0];
        p.left_y  = posL[1];
        p.left_z  = posL[2];
        p.waist        = last_waist_angle;
        p.right_wrist  = wristR;
        p.left_wrist   = wristL;
 
        out.push(p);
    }
 
    return out;
}