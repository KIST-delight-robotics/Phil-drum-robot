#include "trajectory/base_motion_generator.hpp"

#include <algorithm>

BaseMotionGenerator::BaseMotionGenerator() {

}

BaseMotionGenerator::~BaseMotionGenerator() {

}

void BaseMotionGenerator::initialize(const std::map<int, InstrumentCoordinate>& coordinates) {
 
    solver.initialize();
    drum_coordinates = coordinates;

    // 후보점이 없는 악기는 대표점 1개가 유일 후보 (선택 로직이 즉시 반환 → 기존 동작과 동일)
    for (auto& [instrument, coord] : drum_coordinates) {
        if (coord.right_candidate_positions.empty()) coord.right_candidate_positions = {coord.right_position};
        if (coord.left_candidate_positions.empty())  coord.left_candidate_positions  = {coord.left_position};
    }

    // 후보점별 팔 단독 허리 가능 집합 사전 계산 (핫 리로드마다 재계산)
    candidate_waist_feasibility = {};
    int num_candidates = 0;
    for (const auto& [instrument, coord] : drum_coordinates) {
        for (const auto& position : coord.right_candidate_positions) {
            candidate_waist_feasibility[0][position] = sweep_arm_waist_feasibility(Arm::RIGHT, position, coord.right_wrist_angle);
            num_candidates++;
        }
        for (const auto& position : coord.left_candidate_positions) {
            candidate_waist_feasibility[1][position] = sweep_arm_waist_feasibility(Arm::LEFT, position, coord.left_wrist_angle);
            num_candidates++;
        }
    }
    std::cout << "[BaseMotionGenerator] 후보점 " << num_candidates << "개의 허리 가능 집합 계산 완료\n";
}

BaseMotionPoint BaseMotionGenerator::reset(int note_r, int note_l) {
    BaseMotionPoint point;
    note_to_target(note_r, Arm::RIGHT, point.right_position, point.right_wrist);
    note_to_target(note_l, Arm::LEFT, point.left_position, point.left_wrist);

    // 시작 위치: 양팔 동시 허리 가능 집합이 가장 넓은 후보 쌍 (후보가 각 1개면 대표점 = 기존 동작)
    const auto* right_candidates = get_candidate_positions(Arm::RIGHT, note_r);
    const auto* left_candidates  = get_candidate_positions(Arm::LEFT,  note_l);
    if (right_candidates && left_candidates && !right_candidates->empty() && !left_candidates->empty()) {
        const bool same_instrument = (physical_instrument_id(note_r) == physical_instrument_id(note_l));
        size_t best_i = 0, best_j = 0;
        int best_count = -1;
        // pass 0: 같은 악기면 R.x > L.x 쌍만, pass 1: 만족 쌍이 없으면 제약 해제
        for (int pass = 0; pass < 2 && best_count < 0; pass++) {
            for (size_t i = 0; i < right_candidates->size(); i++) {
                for (size_t j = 0; j < left_candidates->size(); j++) {
                    const auto& pR = (*right_candidates)[i];
                    const auto& pL = (*left_candidates)[j];
                    if (pass == 0 && same_instrument && !(pR[0] > pL[0])) continue;
                    int count = (int)(get_arm_waist_feasibility(Arm::RIGHT, pR, point.right_wrist)
                                    & get_arm_waist_feasibility(Arm::LEFT,  pL, point.left_wrist)).count();
                    if (count > best_count) { best_count = count; best_i = i; best_j = j; }
                }
            }
        }
        point.right_position = (*right_candidates)[best_i];
        point.left_position  = (*left_candidates)[best_j];
    }

    right_context = MotionContext{};
    right_context.last_instrument = note_r;
    right_context.last_position   = point.right_position;
    left_context = MotionContext{};
    left_context.last_instrument = note_l;
    left_context.last_position   = point.left_position;

    auto [opt, range] = compute_waist_range(point.right_position, point.left_position, point.right_wrist, point.left_wrist);
    point.waist = opt;

    prev_waist_angle = opt;
    cur_waist_angle = opt;
    prev_t = -1.0;
    cur_q0_min = range[0];
    cur_q0_max = range[1];

    base_end_error = false;

    return point;
}

std::queue<BaseMotionPoint> BaseMotionGenerator::generate_motion(const std::vector<DrumEvent>& rds, int num_point, double dt) {
    std::queue<BaseMotionPoint> out;
 
    if (rds.size() < 2 || num_point <= 0) {
        return out;
    }

    // 오른팔은 왼팔의 직전 계획을, 왼팔은 오른팔의 새 계획을 보고 후보를 고른다 (verbose: 선택 로그)
    MotionSegment seg_R = get_motion_segment(rds, Arm::RIGHT, right_context, make_segment_from_context(Arm::LEFT, left_context), true);
    MotionSegment seg_L = get_motion_segment(rds, Arm::LEFT,  left_context,  seg_R, true);

    WaistSegment seg_w = get_waist_segment(rds);

    right_context = seg_R.next_context; // context update
    left_context = seg_L.next_context;

    // std::cout << "===== R =====\n";
    // std::cout << "start_time: " << seg_R.start_time << "\n";
    // std::cout << "end_time: " << seg_R.end_time << "\n";
    // std::cout << "t0: " << seg_R.t0 << "\n";
    // std::cout << "t1: " << seg_R.t1 << "\n";

    // std::cout << "===== L =====\n";
    // std::cout << "start_time: " << seg_L.start_time << "\n";
    // std::cout << "end_time: " << seg_L.end_time << "\n";
    // std::cout << "t0: " << seg_L.t0 << "\n";
    // std::cout << "t1: " << seg_L.t1 << "\n";

    for (int i = 0; i < num_point; i++) {
        BaseMotionPoint point;

        double t_R = i * dt + seg_R.t0 - seg_R.start_time;
        double t_L = i * dt + seg_L.t0 - seg_L.start_time;

        double s_R = time_scaling(0.0, seg_R.end_time - seg_R.start_time, t_R);
        double s_L = time_scaling(0.0, seg_L.end_time - seg_L.start_time, t_L);
        
        point.right_position = make_path(seg_R.start_position, seg_R.end_position, s_R);
        point.left_position = make_path(seg_L.start_position, seg_L.end_position, s_L);

        point.right_wrist = s_R * (seg_R.end_wrist_angle - seg_R.start_wrist_angle) + seg_R.start_wrist_angle;
        point.left_wrist = s_L * (seg_L.end_wrist_angle - seg_L.start_wrist_angle) + seg_L.start_wrist_angle;

        double t_w = i * dt + seg_w.t0;
        point.waist = cubic_hermite(seg_w.t0, seg_w.q0, seg_w.v0, seg_w.t1, seg_w.q1, seg_w.v1, t_w);

        out.push(point);
    }

    return out;
}

bool BaseMotionGenerator::get_error() {
    return base_end_error;
}

BaseMotionGenerator::MotionSegment BaseMotionGenerator::get_motion_segment(const std::vector<DrumEvent>& rds, Arm arm, const MotionContext& context,
                                                                           const MotionSegment& other_arm_segment, bool verbose) {
    MotionSegment seg;

    seg.t0 = rds[0].t;
    seg.t1 = rds[1].t;

    // 팔에 따라 참조할 note 선택
    auto note_of = [&](int i) {
        int note = (arm == Arm::RIGHT) ? rds[i].note_num_R : rds[i].note_num_L;
        if (note == 5 && !rds[i].is_closed_hihat) note = 9; // 오픈 하이햇 처리
        return note;
    };

    // ===== 타격 감지 =====
    const double e = 0.00001;
    int rds_size = (int)rds.size();

    bool   is_hit   = false;
    double t_hit    = 0.0;
    int    note_hit = 0;

    for (int i = 1; i < rds_size; i++) {
        if (round(10000 * (HIT_DETECTION_THRESHOLD + e)) < round(10000 * (rds[i].t - rds[0].t))) {
            break;
        }

        if (note_of(i) != 0) {
            is_hit   = true;
            t_hit    = rds[i].t;
            note_hit = note_of(i);
            break;
        }
    }

    // ===== 악기 & 시간 =====
    // 위치는 컨텍스트에 잠긴 점을 쓰고, 손목각만 악기별 값으로 조회한다
    // (잘못된 악기 번호의 오류 처리는 note_to_target에 그대로 둔다)
    const bool flying = (context.state == State::REST_TO_HIT || context.state == State::HIT_TO_HIT);
    std::array<double, 3> unused_position;

    auto set_start = [&](int instrument, const std::array<double, 3>& position) {
        note_to_target(instrument, arm, unused_position, seg.start_wrist_angle);
        seg.start_position   = position;
        seg.start_instrument = instrument;
    };
    auto set_end = [&](int instrument, const std::array<double, 3>& position) {
        note_to_target(instrument, arm, unused_position, seg.end_wrist_angle);
        seg.end_position   = position;
        seg.end_instrument = instrument;
    };
    auto hold_position = [&]() {   // 다음 타격 없음: 현재 위치 유지
        seg.end_position    = seg.start_position;
        seg.end_wrist_angle = seg.start_wrist_angle;
        seg.end_instrument  = seg.start_instrument;
    };

    MotionContext next_context = context;

    if (note_of(0) == 0) {
        if (flying) {
            // 비행 지속: 출발 시 잠긴 목표를 그대로 사용 (재선택 없음)
            set_start(context.last_instrument, context.last_position);
            set_end(context.target_instrument, context.target_position);

            seg.start_time = context.last_t;
            seg.end_time   = context.target_time;
        } else if (is_hit) {
            // 휴식 중 다음 타격 찾음 → 비행 시작: 후보 선택 후 잠금
            set_start(context.last_instrument, context.last_position);
            set_end(note_hit, select_hit_position(arm, note_hit, t_hit, other_arm_segment,
                                                  context.last_instrument, context.last_position, verbose));

            seg.start_time = rds[0].t;
            seg.end_time   = t_hit;

            next_context.state             = State::REST_TO_HIT;
            next_context.last_t            = rds[0].t;
            next_context.target_instrument = note_hit;
            next_context.target_position   = seg.end_position;
            next_context.target_time       = t_hit;
        } else {
            // 휴식 중 다음 타격 없음 (현재 위치 유지)
            set_start(context.last_instrument, context.last_position);
            hold_position();

            seg.start_time = rds[0].t;
            seg.end_time   = rds[1].t;

            next_context.state             = State::REST_TO_REST;
            next_context.last_t            = rds[0].t;
            next_context.target_instrument = 0;
        }
    } else {
        // 타격 순간의 위치: 비행 중이던 목표점, 또는 서 있던 점
        std::array<double, 3> arrival_position;
        if (flying && context.target_instrument == note_of(0)) {
            arrival_position = context.target_position;
        } else if (!flying && context.last_instrument == note_of(0)) {
            arrival_position = context.last_position;
        } else {
            // 비행 없이 다른 악기를 치는 경우 (악보상 순간이동): 후보 중 골라 점프
            if (verbose) {
                std::cerr << "[BaseMotionGenerator] 비행 없는 타격: t=" << rds[0].t
                          << " arm=" << (arm == Arm::RIGHT ? "R" : "L")
                          << " instrument " << (flying ? context.target_instrument : context.last_instrument)
                          << " -> " << note_of(0) << "\n";
            }
            arrival_position = select_hit_position(arm, note_of(0), rds[0].t, other_arm_segment, 0, {}, verbose);
        }
        set_start(note_of(0), arrival_position);

        next_context = MotionContext{};
        next_context.last_t          = rds[0].t;
        next_context.last_instrument = note_of(0);
        next_context.last_position   = arrival_position;

        if (is_hit) {
            // 타격 후 다음 타격 찾음 → 비행 시작
            set_end(note_hit, select_hit_position(arm, note_hit, t_hit, other_arm_segment,
                                                  note_of(0), arrival_position, verbose));

            seg.start_time = rds[0].t;
            seg.end_time   = t_hit;

            next_context.state             = State::HIT_TO_HIT;
            next_context.target_instrument = note_hit;
            next_context.target_position   = seg.end_position;
            next_context.target_time       = t_hit;
        } else {
            // 타격 후 다음 타격 없음 (현재 위치 유지)
            hold_position();

            seg.start_time = rds[0].t;
            seg.end_time   = rds[1].t;

            next_context.state = State::HIT_TO_REST;
        }
    }

    seg.next_context = next_context;

    return seg;
}

double BaseMotionGenerator::get_wrist_angle(Arm arm, int instrument) {
    auto it = drum_coordinates.find(instrument);
    if (it == drum_coordinates.end()) return 0.0;
    return (arm == Arm::RIGHT) ? it->second.right_wrist_angle : it->second.left_wrist_angle;
}

const std::vector<std::array<double, 3>>* BaseMotionGenerator::get_candidate_positions(Arm arm, int instrument) {
    auto it = drum_coordinates.find(instrument);
    if (it == drum_coordinates.end()) return nullptr;
    return (arm == Arm::RIGHT) ? &it->second.right_candidate_positions : &it->second.left_candidate_positions;
}

BaseMotionGenerator::WaistFeasibility BaseMotionGenerator::sweep_arm_waist_feasibility(Arm arm, const std::array<double, 3>& position, double wrist_angle) {
    WaistFeasibility feasibility;
    const auto side = (arm == Arm::RIGHT) ? KinematicsSolver::ArmSide::RIGHT : KinematicsSolver::ArmSide::LEFT;

    for (int i = 0; i < NUM_WAIST_SAMPLES; i++) {
        const double theta0 = waist_sample_angle(i);
        feasibility[i] = solver.check_joint_limit(0, theta0)
                      && solver.solve_arm_ik(position, theta0, wrist_angle, side, false).success;
    }
    return feasibility;
}

BaseMotionGenerator::WaistFeasibility BaseMotionGenerator::get_arm_waist_feasibility(Arm arm, const std::array<double, 3>& position, double wrist_angle) {
    const auto& cache = candidate_waist_feasibility[(arm == Arm::RIGHT) ? 0 : 1];
    auto it = cache.find(position);
    if (it != cache.end()) return it->second;

    // 비행 중 보간점 (후보가 아님): 스윕만 하고 캐시에는 넣지 않는다
    return sweep_arm_waist_feasibility(arm, position, wrist_angle);
}

BaseMotionGenerator::MotionSegment BaseMotionGenerator::make_segment_from_context(Arm arm, const MotionContext& context) {
    MotionSegment seg{};
    const bool flying = (context.state == State::REST_TO_HIT || context.state == State::HIT_TO_HIT);

    seg.start_position    = context.last_position;
    seg.start_wrist_angle = get_wrist_angle(arm, context.last_instrument);
    seg.start_instrument  = context.last_instrument;
    seg.start_time        = context.last_t;

    if (flying) {
        seg.end_position    = context.target_position;
        seg.end_wrist_angle = get_wrist_angle(arm, context.target_instrument);
        seg.end_instrument  = context.target_instrument;
        seg.end_time        = context.target_time;
    } else {
        seg.end_position    = seg.start_position;
        seg.end_wrist_angle = seg.start_wrist_angle;
        seg.end_instrument  = seg.start_instrument;
        seg.end_time        = seg.start_time;
    }

    seg.t0 = seg.start_time;
    seg.t1 = seg.end_time;
    return seg;
}

std::array<double, 3> BaseMotionGenerator::select_hit_position(Arm arm, int instrument, double hit_time,
                                                               const MotionSegment& other_arm_segment,
                                                               int start_instrument, const std::array<double, 3>& start_position,
                                                               bool verbose) {
    const auto* candidates = get_candidate_positions(arm, instrument);
    if (!candidates || candidates->empty()) {
        return {0.0, 0.0, 0.0};   // 잘못된 악기 번호: 호출 측 note_to_target이 오류 플래그를 세운다
    }
    if (candidates->size() == 1) {
        return (*candidates)[0];  // 후보 1개 (대표점 폴백): 선택 없음
    }

    const Arm    other_arm = (arm == Arm::RIGHT) ? Arm::LEFT : Arm::RIGHT;
    const double my_wrist  = get_wrist_angle(arm, instrument);
    const auto&  other     = other_arm_segment;

    // 1) 반대손의 내 타격 시각 위치 → 반대팔 단독 허리 가능 집합 (실제 궤적과 같은 time_scaling + make_path)
    const double s = time_scaling(0.0, other.end_time - other.start_time, hit_time - other.start_time);
    const std::array<double, 3> other_position_at_hit = make_path(other.start_position, other.end_position, s);
    const double other_wrist_at_hit = s * (other.end_wrist_angle - other.start_wrist_angle) + other.start_wrist_angle;
    const WaistFeasibility other_arm_feasibility = get_arm_waist_feasibility(other_arm, other_position_at_hit, other_wrist_at_hit);

    auto feasible_waist_count = [&](const std::array<double, 3>& candidate) {
        return (int)(get_arm_waist_feasibility(arm, candidate, my_wrist) & other_arm_feasibility).count();
    };

    // 2) 같은 물리 악기 위의 반대손과 좌우 순서 제약 (R.x > L.x). 기준점: 반대손 도착점, 아직 도착 전이면 출발점도
    std::vector<std::array<double, 3>> hand_order_anchors;
    if (physical_instrument_id(other.end_instrument) == physical_instrument_id(instrument)) {
        hand_order_anchors.push_back(other.end_position);
    }
    if (s < 1.0 && physical_instrument_id(other.start_instrument) == physical_instrument_id(instrument)) {
        hand_order_anchors.push_back(other.start_position);
    }
    auto satisfies_hand_order = [&](const std::array<double, 3>& candidate) {
        for (const auto& anchor : hand_order_anchors) {
            const bool ok = (arm == Arm::RIGHT) ? (candidate[0] > anchor[0]) : (candidate[0] < anchor[0]);
            if (!ok) return false;
        }
        return true;
    };
    const bool use_hand_order = std::any_of(candidates->begin(), candidates->end(), satisfies_hand_order);   // 만족 후보가 없으면 제약 해제
    auto allowed = [&](const std::array<double, 3>& candidate) { return !use_hand_order || satisfies_hand_order(candidate); };

    // 3) 같은 악기 반복 타격: 현 후보 유지 (순서 제약 만족 + 폭 > 0 일 때)
    int  chosen       = -1;
    bool keep_current = false;
    if (start_instrument != 0 && physical_instrument_id(start_instrument) == physical_instrument_id(instrument)) {
        const auto* start_candidates = get_candidate_positions(arm, start_instrument);
        if (start_candidates) {
            auto it = std::find(start_candidates->begin(), start_candidates->end(), start_position);
            const size_t k = it - start_candidates->begin();   // closed/open hihat은 리스트가 z만 달라 같은 인덱스가 대응
            if (it != start_candidates->end() && k < candidates->size()
                && allowed((*candidates)[k]) && feasible_waist_count((*candidates)[k]) > 0) {
                chosen = (int)k;
                keep_current = true;
            }
        }
    }

    // 4) 허리 가능 폭 최대 후보 (동률: 낮은 인덱스)
    if (chosen < 0) {
        int best_count = -1;
        for (size_t i = 0; i < candidates->size(); i++) {
            if (!allowed((*candidates)[i])) continue;
            const int count = feasible_waist_count((*candidates)[i]);
            if (count > best_count) { best_count = count; chosen = (int)i; }
        }
        if (best_count == 0) {
            // 반대손 위치와 양립하는 허리각이 없음: 내 팔 단독 폭 최대로 폴백 (이후 compute_waist_range가 오류 처리)
            for (size_t i = 0; i < candidates->size(); i++) {
                const int count = (int)get_arm_waist_feasibility(arm, (*candidates)[i], my_wrist).count();
                if (count > best_count) { best_count = count; chosen = (int)i; }
            }
        }
    }

    if (verbose) {
        std::cerr << "[BaseMotionGenerator] select t=" << hit_time
                  << " arm=" << (arm == Arm::RIGHT ? "R" : "L")
                  << " instrument=" << instrument
                  << " candidate=" << chosen << "/" << candidates->size()
                  << " position=[" << (*candidates)[chosen][0] << ", " << (*candidates)[chosen][1] << ", " << (*candidates)[chosen][2] << "]"
                  << " waist_width=" << feasible_waist_count((*candidates)[chosen]) * 0.1 << "deg"
                  << " keep_current=" << keep_current
                  << " hand_order=" << hand_order_anchors.size()
                  << " | other instrument=" << other.end_instrument
                  << ((s > 0.0 && s < 1.0) ? " flying" : " rest") << "\n";
    }

    return (*candidates)[chosen];
}

void BaseMotionGenerator::note_to_target(int note_num, Arm arm, std::array<double, 3>& out_position, double& out_wrist_angle_deg) {
    auto it = drum_coordinates.find(note_num);
    if (it == drum_coordinates.end()) {
        std::cerr << "[BaseMotionGenerator] note_to_target: invalid note number "
                  << note_num << "\n";
        out_position = {0.0, 0.0, 0.0};
        out_wrist_angle_deg = 0.0;
        base_end_error = true;  // 악보 오류: 연주 종료
        return;
    }
    const InstrumentCoordinate& coord = it->second;
    if (arm == Arm::RIGHT) { out_position = coord.right_position; out_wrist_angle_deg = coord.right_wrist_angle; }
    else                   { out_position = coord.left_position;  out_wrist_angle_deg = coord.left_wrist_angle; }
}

double BaseMotionGenerator::time_scaling(double ti, double tf, double t) {
    if (tf <= ti) {
        return (t < ti) ? 0.0 : 1.0;   // 0으로 나누기 방지
    }

    double s = (t - ti) / (tf - ti);

    // 구간 밖 클램프
    if (s <= 0.0) return 0.0;
    if (s >= 1.0) return 1.0;

    return s * s * (3.0 - 2.0 * s);   // 3s^2 - 2s^3
}

std::array<double, 3> BaseMotionGenerator::make_path(const std::array<double, 3>& pi, const std::array<double, 3>& pf, double s) {
    std::array<double, 3> ps;

    if (pi == pf) {
        ps = pi;
    } else {
        double h1 = 0.0, h2 = 0.08 + 0.6 * std::abs(pf[2] - pi[2]);     // 조절할 수 있음
        
        std::array<double, 3> pm1;
        std::array<double, 3> pm2;
        for (int i = 0; i < 2; i++) {
            pm1[i] = (2.0/3.0) * pi[i] + (1.0/3.0) * pf[i];
            pm2[i] = (1.0/3.0) * pi[i] + (2.0/3.0) * pf[i];
        }
        pm1[2] = pi[2] + h1;
        pm2[2] = pf[2] + h2;

        // 3차 Bézier curve
        double u = 1.0 - s;
        double b0 = u*u*u;
        double b1 = 3.0*u*u*s;
        double b2 = 3.0*u*s*s;
        double b3 = s*s*s;
        for (int i = 0; i < 3; i++) {
            ps[i] = b0*pi[i] + b1*pm1[i] + b2*pm2[i] + b3*pf[i];
        }
    }

    return ps;
}

BaseMotionGenerator::WaistSegment BaseMotionGenerator::get_waist_segment(const std::vector<DrumEvent>& rds) {
    std::array<double, 4> t_03{};
    std::array<double, 4> q0_opt{};
    std::array<double, 4> q0_min{};
    std::array<double, 4> q0_max{};

    t_03[0] = rds[0].t;
    q0_opt[0] = cur_waist_angle;
    q0_min[0] = cur_q0_min;
    q0_max[0] = cur_q0_max;

    for (int i = 1; i < 4; i++) {
        if ((int)rds.size() > i) {
            t_03[i] = rds[i].t;
            auto [opt, range] = get_waist_angle(rds, i);
            q0_opt[i] = opt;
            q0_min[i] = range[0];
            q0_max[i] = range[1];
        } else {
            t_03[i] = t_03[i - 1] + 1.0;
            q0_opt[i] = q0_opt[i - 1];
            q0_min[i] = q0_min[i - 1];
            q0_max[i] = q0_max[i - 1];
        }
    }

    // 기울기 평균 이동: t0 -> t1
    std::array<double, 3> a0{};
    for (int i = 0; i < 3; i++) {
        a0[i] = (q0_opt[i + 1] - cur_waist_angle) / (t_03[i + 1] - t_03[0]);
    }
    double avg_a0 = (a0[0] + a0[1] + a0[2]) / 3.0;
    double next_waist_angle = cur_waist_angle + avg_a0 * (t_03[1] - t_03[0]);

    if (next_waist_angle <= q0_min[1] || next_waist_angle >= q0_max[1]) {
        next_waist_angle = (q0_min[1] + q0_max[1]) / 2.0;
    }

    // 기울기 평균 이동: t1 -> t2
    std::array<double, 2> a1{};
    for (int i = 0; i < 2; i++) {
        a1[i] = (q0_opt[i + 2] - next_waist_angle) / (t_03[i + 2] - t_03[1]);
    }
    double avg_a1 = (a1[0] + a1[1]) / 2.0;
    double next_next_waist_angle = next_waist_angle + avg_a1 * (t_03[2] - t_03[1]);

    if (next_next_waist_angle <= q0_min[2] || next_next_waist_angle >= q0_max[2]) {
        next_next_waist_angle = (q0_min[2] + q0_max[2]) / 2.0;
    }

    // 기울기 계산
    std::array<double, 4> q = {prev_waist_angle, cur_waist_angle, next_waist_angle, next_next_waist_angle};
    std::array<double, 4> t = {prev_t, t_03[0], t_03[1], t_03[2]};

    std::array<double, 2> m = compute_slopes(q, t);

    // 반환
    WaistSegment seg;

    seg.t0 = t_03[0];
    seg.t1 = t_03[1];
    
    seg.q0 = cur_waist_angle;
    seg.q1 = next_waist_angle;

    seg.v0 = m[0];
    seg.v1 = m[1];

    prev_t = t_03[0];
    prev_waist_angle = cur_waist_angle;
    cur_waist_angle = next_waist_angle;
    cur_q0_min = q0_min[1];
    cur_q0_max = q0_max[1];

    return seg;
}

std::pair<double, std::array<double, 2>> BaseMotionGenerator::get_waist_angle(const std::vector<DrumEvent>& rds, int idx) {
    // idx 번째 허리 최적값, 범위 구하기

    MotionContext tmp_context_R = right_context;
    MotionContext tmp_context_L = left_context;

    for (int i = 0; i < (int)rds.size() - 1; i++) {
        std::vector<DrumEvent> rds_from_i(rds.begin() + i, rds.end());
        MotionSegment seg_R = get_motion_segment(rds_from_i, Arm::RIGHT, tmp_context_R, make_segment_from_context(Arm::LEFT, tmp_context_L));
        MotionSegment seg_L = get_motion_segment(rds_from_i, Arm::LEFT,  tmp_context_L, seg_R);

        if (i + 1 == idx) {
            double t_R = seg_R.t1 - seg_R.start_time;
            double t_L = seg_L.t1 - seg_L.start_time;

            double s_R = time_scaling(0.0, seg_R.end_time - seg_R.start_time, t_R);
            double s_L = time_scaling(0.0, seg_L.end_time - seg_L.start_time, t_L);
            
            std::array<double, 3> right_position = make_path(seg_R.start_position, seg_R.end_position, s_R);
            std::array<double, 3> left_position = make_path(seg_L.start_position, seg_L.end_position, s_L);

            double right_wrist = s_R * (seg_R.end_wrist_angle - seg_R.start_wrist_angle) + seg_R.start_wrist_angle;
            double left_wrist = s_L * (seg_L.end_wrist_angle - seg_L.start_wrist_angle) + seg_L.start_wrist_angle;

            return compute_waist_range(right_position, left_position, right_wrist, left_wrist); 
        }

        tmp_context_R = seg_R.next_context;
        tmp_context_L = seg_L.next_context;
    }

    base_end_error = true;  // 인덱싱 오류: 연주 종료
    std::array<double, 2> err{};
    return {0.0, err};
}

std::pair<double, std::array<double, 2>> BaseMotionGenerator::compute_waist_range(std::array<double, 3> pR, std::array<double, 3> pL, double the7, double the8) {
    std::vector<std::array<double, 9>> q_vec;
    int num_sol = 0;

    for (int i = 0; i < NUM_WAIST_SAMPLES; i++) {
        double the0 = waist_sample_angle(i);  // 범위 : -90deg ~ 90deg

        KinematicsSolver::IKResult result = solver.solve_ik(pR, pL, the0, the7, the8, false);

        if (result.success) {
            q_vec.push_back(result.q);
            num_sol++;
        }
    }

    double w0 = 2.0, w1 = 1.0;
    double min_cost = w0 + w1;
    int min_idx = 0;
    double w, cost = 0.0;

    if (num_sol == 0) {
        std::cerr << "[BaseMotionGenerator] 허리 -90~90deg 전 범위에서 IK 해 없음\n";
        base_end_error = true;  // IK 오류: 연주 종료
        std::array<double, 2> err{};
        return {0.0, err};
    } else {
        std::array<double, 2> range = {q_vec[0][0], q_vec[num_sol - 1][0]};
        w = 2.0 * M_PI / std::abs(range[1] - range[0]);

        for (int i = 0; i < num_sol; i++) {
            // 양 팔의 관절각 합이 180도에 가까운 값 선택 + solution set 중 가운데 값 선택
            cost = w0 * cos(q_vec[i][1] + q_vec[i][2]) + w1 * cos(w * std::abs(q_vec[i][0] - range[0]));

            if (cost < min_cost) {
                min_cost = cost;
                min_idx = i;
            }
        }

        return {q_vec[min_idx][0], range};
    }
}

std::array<double, 2> BaseMotionGenerator::compute_slopes(const std::array<double, 4> &q, const std::array<double, 4> &t) {
    // Monotone Cubic Interpolation 을 위한 기울기 계산
    std::array<double, 2> m{};
    std::array<double, 3> a{};

    for (int i = 0; i < 3; i++) {
        a[i] = (q[i + 1] - q[i]) / (t[i + 1] - t[i]);
    }
    double m1 = 0.5 * (a[0] + a[1]);
    double m2 = 0.5 * (a[1] + a[2]);

    double alph, bet;
    if (q[1] == q[2]) {
        m1 = 0;
        m2 = 0;
    } else {
        if((q[0] == q[1]) || (a[0] * a[1] < 0)){
            m1 = 0;
        } else if((q[2] == q[3]) || (a[1] * a[2] < 0)){
            m2 = 0;
        }
        alph = m1 / (q[2] - q[1]);
        bet = m2 / (q[2] - q[1]);

        double e = std::sqrt(std::pow(alph, 2) + std::pow(bet, 2));
        if (e > 3.0) {
            m1 = (3 * m1) / e;
            m2 = (3 * m2) / e;
        }
    }
    
    m[0] = m1;
    m[1] = m2;
    
    return m;
}

double BaseMotionGenerator::cubic_hermite(double ta, double qa, double va, double tb, double qb, double vb, double t) {
    // [ta,tb] 구간을 q(ta)=qa, q'(ta)=va, q(tb)=qb, q'(tb)=vb 로 보간하는 3차 다항식.
    double T = tb - ta;
    if (T == 0.0) {
        return qa;
    }
    double tau = (t - ta) / T;      // 0 ~ 1

    double h00 =  2.0*tau*tau*tau - 3.0*tau*tau + 1.0;
    double h10 =      tau*tau*tau - 2.0*tau*tau + tau;
    double h01 = -2.0*tau*tau*tau + 3.0*tau*tau;
    double h11 =      tau*tau*tau -     tau*tau;

    return h00*qa + h10*T*va + h01*qb + h11*T*vb;
}