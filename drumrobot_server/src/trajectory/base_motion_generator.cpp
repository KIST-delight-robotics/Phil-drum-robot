#include "trajectory/base_motion_generator.hpp"

#include <algorithm>

BaseMotionGenerator::BaseMotionGenerator() {

}

BaseMotionGenerator::~BaseMotionGenerator() {

}

void BaseMotionGenerator::initialize(const std::map<int, InstrumentCoordinate>& coordinates) {
 
    solver.initialize();
    drum_coordinates = coordinates;

    // 팔별 타격점이 비어 있으면(로더를 거치지 않은 맵) 중심 ± x 오프셋 1개가 유일 후보 (선택 로직이 즉시 반환)
    for (auto& [instrument, coord] : drum_coordinates) {
        std::array<double, 3> right = coord.center, left = coord.center;
        right[0] += ROBOT::CANDIDATE_HAND_X_OFFSET;
        left[0]  -= ROBOT::CANDIDATE_HAND_X_OFFSET;
        if (coord.right_candidate_positions.empty()) coord.right_candidate_positions = {right};
        if (coord.left_candidate_positions.empty())  coord.left_candidate_positions  = {left};
    }

    // 후보점별 팔 단독 허리 가능 집합 사전 계산 (핫 리로드마다 재계산)
    // trajectory_generator.cpp:reload_drum_coordinates() -> play_motion_generator.initialize() -> base_motion_generator.initialize()
    candidate_waist_feasibility = {};
    int num_candidates = 0;
    for (const auto& [instrument, coord] : drum_coordinates) {
        for (const auto& position : coord.right_candidate_positions) {
            candidate_waist_feasibility[0][position] = compute_feasible_waist_range(Arm::RIGHT, position, coord.wrist_angle);
            num_candidates++;
        }
        for (const auto& position : coord.left_candidate_positions) {
            candidate_waist_feasibility[1][position] = compute_feasible_waist_range(Arm::LEFT, position, coord.wrist_angle);
            num_candidates++;
        }
    }
    std::cout << "[BaseMotionGenerator] 후보점 " << num_candidates << "개의 허리 가능 집합 계산 완료\n";
}

BaseMotionPoint BaseMotionGenerator::reset(int note_r, int note_l) {
    BaseMotionPoint point{};
    point.right_wrist = get_wrist_angle(note_r);
    point.left_wrist  = get_wrist_angle(note_l);

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
                    int count = (int)(get_feasible_waist_range(Arm::RIGHT, pR, point.right_wrist)
                                    & get_feasible_waist_range(Arm::LEFT,  pL, point.left_wrist)).count();
                    if (count > best_count) { best_count = count; best_i = i; best_j = j; }
                }
            }
        }
        point.right_position = (*right_candidates)[best_i];
        point.left_position  = (*left_candidates)[best_j];
    }

    right_context = MotionContext{};
    right_context.last_start_instrument = note_r;
    right_context.last_end_instrument   = note_r;
    right_context.last_start_position   = point.right_position;
    right_context.last_end_position     = point.right_position;
    left_context = MotionContext{};
    left_context.last_start_instrument = note_l;
    left_context.last_end_instrument   = note_l;
    left_context.last_start_position   = point.left_position;
    left_context.last_end_position     = point.left_position;

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

    std::pair<MotionSegment, MotionSegment> segments = get_motion_segments(rds, right_context, left_context, true);   // verbose: 선택 로그
    MotionSegment seg_R = segments.first;
    MotionSegment seg_L = segments.second;

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

std::pair<BaseMotionGenerator::MotionSegment, BaseMotionGenerator::MotionSegment> BaseMotionGenerator::get_motion_segments(const std::vector<DrumEvent>& rds, const MotionContext& context_R, const MotionContext& context_L, bool verbose) {
    MotionSegment seg_R;
    MotionSegment seg_L;

    seg_R.t0 = rds[0].t;    seg_L.t0 = rds[0].t;
    seg_R.t1 = rds[1].t;    seg_L.t1 = rds[1].t;

    // 팔에 따라 참조할 note 선택
    auto note_of_R = [&](int i) {
        int note = rds[i].note_num_R;
        if (note == 5 && !rds[i].is_closed_hihat) note = 9; // 오픈 하이햇 처리
        return note;
    };
    auto note_of_L = [&](int i) {
        int note = rds[i].note_num_L;
        if (note == 5 && !rds[i].is_closed_hihat) note = 9; // 오픈 하이햇 처리
        return note;
    };

    // ===== 양팔 개별로 타격 감지 =====
    const double e = 0.00001;
    int rds_size = (int)rds.size();

    bool   find_hit_R = false, find_hit_L = false;
    double t_hit_R = 0.0, t_hit_L = 0.0;
    int    note_hit_R = 0, note_hit_L = 0;

    for (int i = 1; i < rds_size; i++) {
        if (round(10000 * (HIT_DETECTION_THRESHOLD + e)) < round(10000 * (rds[i].t - rds[0].t))) {
            break;
        }

        if (note_of_R(i) != 0) {
            find_hit_R   = true;
            t_hit_R    = rds[i].t;
            note_hit_R = note_of_R(i);
            break;
        }
    }
    for (int i = 1; i < rds_size; i++) {
        if (round(10000 * (HIT_DETECTION_THRESHOLD + e)) < round(10000 * (rds[i].t - rds[0].t))) {
            break;
        }

        if (note_of_L(i) != 0) {
            find_hit_L   = true;
            t_hit_L    = rds[i].t;
            note_hit_L = note_of_L(i);
            break;
        }
    }

    // find_hit_RL  : 현재 함수 실행에서 타격을 찾았는지
    // found_hit_RL : 이전 실행에서 타격을 찾았는지
    bool found_hit_L = (context_L.state == State::REST_TO_HIT || context_L.state == State::HIT_TO_HIT);
    bool found_hit_R = (context_R.state == State::REST_TO_HIT || context_R.state == State::HIT_TO_HIT);    

    // ===== 악기 & 시간 =====
    // 새 비행 시작 여부 = end_position 선정 필요 (비행 중에는 find_hit 이 잠긴 목표를 다시 찾으므로 제외)
    bool need_select_R = find_hit_R && !(note_of_R(0) == 0 && found_hit_R);     // todo : find_hit_R만 있어도됌(0917)
    bool need_select_L = find_hit_L && !(note_of_L(0) == 0 && found_hit_L);

    MotionContext next_context_R, next_context_L;

    // ===== R =====
    // 이전에 타격을 찾아서 이동중
    if(note_of_R(0) == 0 && found_hit_R)
    {
        // 이전 이동 궤적 유지 (재선정 없음)
        seg_R.start_instrument = context_R.last_start_instrument;
        seg_R.end_instrument   = context_R.last_end_instrument;
        seg_R.start_position   = context_R.last_start_position;
        seg_R.end_position     = context_R.last_end_position;
        seg_R.start_time       = context_R.last_t;
        seg_R.end_time         = t_hit_R;

        next_context_R = context_R;
    }
    // 그 외 상황
    else
    {
        // 지금 서 있는(타격하는) 악기와 점
        int cur_instrument_R = (note_of_R(0) == 0) ? context_R.last_end_instrument : note_of_R(0);
        std::array<double, 3> cur_position_R = context_R.last_end_position;
        if(note_of_R(0) != 0 && note_of_R(0) != context_R.last_end_instrument)
        {
            // 비행 없이 다른 악기를 치는 경우 (악보상 순간이동): 첫 후보로 점프. 경고 경로라 후보 선정은 안 한다
            if(verbose) std::cerr << "[BaseMotionGenerator] 비행 없는 타격: t=" << rds[0].t << " arm=R instrument "
                                  << context_R.last_end_instrument << " -> " << note_of_R(0) << "\n";
            if (const auto* c = get_candidate_positions(Arm::RIGHT, note_of_R(0))) cur_position_R = c->front();
        }

        seg_R.start_instrument = cur_instrument_R;
        seg_R.start_position   = cur_position_R;
        seg_R.start_time       = rds[0].t;

        next_context_R.last_t                = rds[0].t;
        next_context_R.last_start_instrument = cur_instrument_R;
        next_context_R.last_start_position   = cur_position_R;

        if(!need_select_R)
        {
            // 현 위치 유지
            seg_R.end_instrument = cur_instrument_R;
            seg_R.end_position   = cur_position_R;
            seg_R.end_time       = rds[1].t;

            next_context_R.state               = (note_of_R(0) == 0) ? State::REST_TO_REST : State::HIT_TO_REST;
            next_context_R.last_end_instrument = cur_instrument_R;
            next_context_R.last_end_position   = cur_position_R;
        }
        else
        {
            // 새 비행 시작: end_position 은 아래 select() 뒤에 채운다
            seg_R.end_instrument = note_hit_R;
            seg_R.end_time       = t_hit_R;

            next_context_R.state               = (note_of_R(0) == 0) ? State::REST_TO_HIT : State::HIT_TO_HIT;
            next_context_R.last_end_instrument = note_hit_R;
            // next_context_R.last_end_position 은 select() 결과
        }
    }

    // ===== L =====
    if(note_of_L(0) == 0 && found_hit_L)
    {
        // 이전 이동 궤적 유지 (재선정 없음)
        seg_L.start_instrument = context_L.last_start_instrument;
        seg_L.end_instrument   = context_L.last_end_instrument;
        seg_L.start_position   = context_L.last_start_position;
        seg_L.end_position     = context_L.last_end_position;
        seg_L.start_time       = context_L.last_t;
        seg_L.end_time         = t_hit_L;

        next_context_L = context_L;
    }
    else
    {
        // 지금 서 있는(타격하는) 악기와 점
        int cur_instrument_L = (note_of_L(0) == 0) ? context_L.last_end_instrument : note_of_L(0);
        std::array<double, 3> cur_position_L = context_L.last_end_position;
        if(note_of_L(0) != 0 && note_of_L(0) != context_L.last_end_instrument)
        {
            // 비행 없이 다른 악기를 치는 경우 (악보상 순간이동): 첫 후보로 점프. 경고 경로라 후보 선정은 안 한다
            if(verbose) std::cerr << "[BaseMotionGenerator] 비행 없는 타격: t=" << rds[0].t << " arm=L instrument "
                                  << context_L.last_end_instrument << " -> " << note_of_L(0) << "\n";
            if (const auto* c = get_candidate_positions(Arm::LEFT, note_of_L(0))) cur_position_L = c->front();
        }

        seg_L.start_instrument = cur_instrument_L;
        seg_L.start_position   = cur_position_L;
        seg_L.start_time       = rds[0].t;

        next_context_L.last_t                = rds[0].t;
        next_context_L.last_start_instrument = cur_instrument_L;
        next_context_L.last_start_position   = cur_position_L;

        if(!need_select_L)
        {
            // 현 위치 유지
            seg_L.end_instrument = cur_instrument_L;
            seg_L.end_position   = cur_position_L;
            seg_L.end_time       = rds[1].t;

            next_context_L.state               = (note_of_L(0) == 0) ? State::REST_TO_REST : State::HIT_TO_REST;
            next_context_L.last_end_instrument = cur_instrument_L;
            next_context_L.last_end_position   = cur_position_L;
        }
        else
        {
            // 새 비행 시작: end_position 은 아래 select() 뒤에 채운다
            seg_L.end_instrument = note_hit_L;
            seg_L.end_time       = t_hit_L;

            next_context_L.state               = (note_of_L(0) == 0) ? State::REST_TO_HIT : State::HIT_TO_HIT;
            next_context_L.last_end_instrument = note_hit_L;
            // next_context_L.last_end_position 은 select() 결과
        }
    }

    // ===== 손목각 =====
    // select_hit_position 이 반대손의 내 타격 시각 손목각을 보간하므로 select() 앞에서 채운다
    seg_R.start_wrist_angle = get_wrist_angle(seg_R.start_instrument);
    seg_R.end_wrist_angle   = get_wrist_angle(seg_R.end_instrument);
    seg_L.start_wrist_angle = get_wrist_angle(seg_L.start_instrument);
    seg_L.end_wrist_angle   = get_wrist_angle(seg_L.end_instrument);

    // ===== end_position 선정 =====
    if(need_select_R || need_select_L)
    {
        auto [end_R, end_L] = select_end_positions(seg_R, seg_L, need_select_R, need_select_L, verbose);
        if(need_select_R) { seg_R.end_position = end_R;  next_context_R.last_end_position = end_R; }
        if(need_select_L) { seg_L.end_position = end_L;  next_context_L.last_end_position = end_L; }
    }

    seg_R.next_context = next_context_R;
    seg_L.next_context = next_context_L;

    return std::pair<MotionSegment, MotionSegment>(seg_R, seg_L);
}

double BaseMotionGenerator::get_wrist_angle(int instrument) {
    auto it = drum_coordinates.find(instrument);
    if (it == drum_coordinates.end()) return 0.0;
    return it->second.wrist_angle;
}

// ===== 팔별 타격점 선정 기준 관련  =====

const std::vector<std::array<double, 3>>* BaseMotionGenerator::get_candidate_positions(Arm arm, int instrument) {
    // 인자로 받은 arm쪽 후보점 리스트를 반환.
    auto it = drum_coordinates.find(instrument);
    if (it == drum_coordinates.end()) return nullptr;
    return (arm == Arm::RIGHT) ? &it->second.right_candidate_positions : &it->second.left_candidate_positions;
}

BaseMotionGenerator::WaistFeasibility BaseMotionGenerator::compute_feasible_waist_range(Arm arm, const std::array<double, 3>& position, double wrist_angle) {
    /* 
    카메라로 드럼을 스캔하여 후보점 생성
    각 후보점에 대해 ROBOT::CANDIDATE_HAND_X_OFFSET 만큼 x 좌표를 이동시킨 각 팔에 대한 후보점을 개별 생성
    right/left_candidate_positions에 저장
    각 팔, 각 후보점에 대해 팔 단독으로 IK 해가 존재하는 범위를 계산하는 함수
    */
    WaistFeasibility feasibility;
    const auto side = (arm == Arm::RIGHT) ? KinematicsSolver::ArmSide::RIGHT : KinematicsSolver::ArmSide::LEFT;

    for (int i = 0; i < NUM_WAIST_SAMPLES; i++) {
        const double theta0 = -0.5 * M_PI + M_PI / 1800.0 * i;      //theta_0 : -90 deg ~ 90 deg
        feasibility[i] = solver.check_joint_limit(0, theta0) && solver.solve_arm_ik(position, theta0, wrist_angle, side, false).success;
    }
    return feasibility;
}

BaseMotionGenerator::WaistFeasibility BaseMotionGenerator::get_feasible_waist_range(Arm arm, const std::array<double, 3>& position, double wrist_angle) {
    // 후보점에 대한 허리 범위 조회
    const auto& cache = candidate_waist_feasibility[(arm == Arm::RIGHT) ? 0 : 1];
    auto it = cache.find(position);
    if (it != cache.end()) return it->second;

    // 비행 중 보간점에 대한 허리 범위 계산
    return compute_feasible_waist_range(arm, position, wrist_angle);
}

BaseMotionGenerator::CandidateScore BaseMotionGenerator::compute_candidate_scores(Arm arm, const MotionSegment& seg, const MotionSegment& other_seg) {
    CandidateScore score;
    const auto* candidates = get_candidate_positions(arm, seg.end_instrument);
    if (!candidates) return score;

    const Arm    other_arm = (arm == Arm::RIGHT) ? Arm::LEFT : Arm::RIGHT;
    const double my_wrist  = get_wrist_angle(seg.end_instrument);
    const auto&  other     = other_seg;

    // 1) 반대손의 내 타격 시각 위치 → 반대팔 단독 허리 가능 집합 (실제 궤적과 같은 time_scaling + make_path)
    const double s = time_scaling(0.0, other.end_time - other.start_time, seg.end_time - other.start_time); // 검증필요
    const std::array<double, 3> other_position_at_hit = make_path(other.start_position, other.end_position, s);
    const double other_wrist_at_hit = s * (other.end_wrist_angle - other.start_wrist_angle) + other.start_wrist_angle;
    const WaistFeasibility other_arm_feasibility = get_feasible_waist_range(other_arm, other_position_at_hit, other_wrist_at_hit);

    // 2) 같은 물리 악기 위의 반대손과 좌우 순서 제약 (R.x > L.x). 기준점: 반대손 도착점, 아직 도착 전이면 출발점도
    std::vector<std::array<double, 3>> hand_order_anchors;
    if (physical_instrument_id(other.end_instrument) == physical_instrument_id(seg.end_instrument)) {
        hand_order_anchors.push_back(other.end_position);
    }
    if (s < 1.0 && physical_instrument_id(other.start_instrument) == physical_instrument_id(seg.end_instrument)) {
        hand_order_anchors.push_back(other.start_position);
    }

    score.waist_width.reserve(candidates->size());
    score.hand_order_ok.reserve(candidates->size());
    for (const auto& candidate : *candidates) {
        score.waist_width.push_back((int)(get_feasible_waist_range(arm, candidate, my_wrist) & other_arm_feasibility).count());
        bool ok = true;
        for (const auto& anchor : hand_order_anchors) {
            ok = ok && ((arm == Arm::RIGHT) ? (candidate[0] > anchor[0]) : (candidate[0] < anchor[0]));
        }
        score.hand_order_ok.push_back(ok);
    }

    // 3) 같은 악기 반복 타격: 현 후보 인덱스 (closed/open hihat은 리스트가 z만 달라 같은 인덱스가 대응)
    if (seg.start_instrument != 0 && physical_instrument_id(seg.start_instrument) == physical_instrument_id(seg.end_instrument)) {
        const auto* start_candidates = get_candidate_positions(arm, seg.start_instrument);
        if (start_candidates) {
            auto it = std::find(start_candidates->begin(), start_candidates->end(), seg.start_position);
            const size_t k = it - start_candidates->begin();
            if (it != start_candidates->end() && k < candidates->size()) score.keep_idx = (int)k;
        }
    }
    return score;
}

std::array<double, 3> BaseMotionGenerator::select_hit_position(Arm arm, const MotionSegment& seg, const MotionSegment& other_seg, bool verbose) {
    const auto* candidates = get_candidate_positions(arm, seg.end_instrument);
    if (!candidates || candidates->empty()) {
        return {0.0, 0.0, 0.0};   // 좌표 없는 악기: 악보 파서(0~8)와 좌표 로더(1~9 필수)가 막으므로 정상 경로에선 오지 않는다
    }
    if (candidates->size() == 1) {
        return (*candidates)[0];  // 후보 1개 (대표점 폴백): 선택 없음
    }

    const CandidateScore score = compute_candidate_scores(arm, seg, other_seg);
    const size_t n = candidates->size();

    // 좌우 순서 제약: 만족 후보가 없으면 해제
    const bool use_hand_order = std::any_of(score.hand_order_ok.begin(), score.hand_order_ok.end(), [](bool ok) { return ok; });
    auto allowed = [&](size_t i) { return !use_hand_order || score.hand_order_ok[i]; };

    // 같은 악기 반복 타격: 현 후보 유지 (순서 제약 만족 + 폭 > 0 일 때)
    int  chosen       = -1;
    bool keep_current = false;
    if (score.keep_idx >= 0 && allowed(score.keep_idx) && score.waist_width[score.keep_idx] > 0) {
        chosen       = score.keep_idx;
        keep_current = true;
    }

    // 허리 가능 폭 최대 후보 (동률: 낮은 인덱스)
    if (chosen < 0) {
        int best_count = -1;
        for (size_t i = 0; i < n; i++) {
            if (!allowed(i)) continue;
            if (score.waist_width[i] > best_count) { best_count = score.waist_width[i]; chosen = (int)i; }
        }
        if (best_count == 0) {
            // 반대손 위치와 양립하는 허리각이 없음: 내 팔 단독 폭 최대로 폴백 (이후 compute_waist_range가 오류 처리)
            const double my_wrist = get_wrist_angle(seg.end_instrument);
            for (size_t i = 0; i < n; i++) {
                const int count = (int)get_feasible_waist_range(arm, (*candidates)[i], my_wrist).count();
                if (count > best_count) { best_count = count; chosen = (int)i; }
            }
        }
    }

    if (verbose) {
        const double s = time_scaling(0.0, other_seg.end_time - other_seg.start_time, seg.end_time - other_seg.start_time);
        std::cerr << "[BaseMotionGenerator] select t=" << seg.end_time
                  << " arm=" << (arm == Arm::RIGHT ? "R" : "L")
                  << " instrument=" << seg.end_instrument
                  << " candidate=" << chosen << "/" << n
                  << " position=[" << (*candidates)[chosen][0] << ", " << (*candidates)[chosen][1] << ", " << (*candidates)[chosen][2] << "]"
                  << " waist_width=" << score.waist_width[chosen] * 0.1 << "deg"
                  << " keep_current=" << keep_current
                  << " hand_order=" << use_hand_order
                  << " | other instrument=" << other_seg.end_instrument
                  << ((s > 0.0 && s < 1.0) ? " flying" : " rest") << "\n";
    }

    return (*candidates)[chosen];
}

std::pair<std::array<double, 3>, std::array<double, 3>> BaseMotionGenerator::select_end_positions(const MotionSegment& seg_R, const MotionSegment& seg_L,
                                                                                    bool need_select_R, bool need_select_L, bool verbose) {
    std::array<double, 3> end_R = seg_R.end_position;   // 선정 안 하는 팔은 그대로 돌려준다
    std::array<double, 3> end_L = seg_L.end_position;

    const auto* candidates_R = get_candidate_positions(Arm::RIGHT, seg_R.end_instrument);
    const auto* candidates_L = get_candidate_positions(Arm::LEFT,  seg_L.end_instrument);
    const size_t n_R = candidates_R ? candidates_R->size() : 0;
    const size_t n_L = candidates_L ? candidates_L->size() : 0;

    bool pending_R = need_select_R;
    bool pending_L = need_select_L;

    // 후보가 0~1개인 팔은 선택의 여지가 없다 → 먼저 확정하고 선택 대상에서 제외 (select_hit_position 의 즉시 반환 경로)
    MotionSegment fixed_R = seg_R, fixed_L = seg_L;
    if (pending_R && n_R <= 1) { end_R = fixed_R.end_position = select_hit_position(Arm::RIGHT, seg_R, seg_L, verbose); pending_R = false; }
    if (pending_L && n_L <= 1) { end_L = fixed_L.end_position = select_hit_position(Arm::LEFT,  seg_L, seg_R, verbose); pending_L = false; }

    // 한 손만: 상대는 확정 → 기존 단일 선정 그대로
    if (pending_R != pending_L) {
        if (pending_R) end_R = select_hit_position(Arm::RIGHT, seg_R, fixed_L, verbose);
        else               end_L = select_hit_position(Arm::LEFT,  seg_L, fixed_R, verbose);
        return {end_R, end_L};
    }
    if (!pending_R) return {end_R, end_L};

    // ===== 양손 쌍 탐색 =====
    // 상대 후보 j 마다 내 후보 전체 평가 → 스윕은 N+M 회 (동시 타격이면 보간점 = 후보 → 캐시 히트 0회)
    std::vector<CandidateScore> scores_R(n_L);   // scores_R[j].waist_width[i] : L 이 j 로 갈 때 R 후보 i 의 폭
    std::vector<CandidateScore> scores_L(n_R);   // scores_L[i].waist_width[j] : R 이 i 로 갈 때 L 후보 j 의 폭
    MotionSegment tmp_R = seg_R, tmp_L = seg_L;
    for (size_t j = 0; j < n_L; j++) { tmp_L.end_position = (*candidates_L)[j]; scores_R[j] = compute_candidate_scores(Arm::RIGHT, seg_R, tmp_L); }
    for (size_t i = 0; i < n_R; i++) { tmp_R.end_position = (*candidates_R)[i]; scores_L[i] = compute_candidate_scores(Arm::LEFT,  seg_L, tmp_R); }

    auto pair_width = [&](size_t i, size_t j) { return std::min(scores_R[j].waist_width[i], scores_L[i].waist_width[j]); };   // 허리는 좁은 손이 병목
    auto pair_order = [&](size_t i, size_t j) { return scores_R[j].hand_order_ok[i] && scores_L[i].hand_order_ok[j]; };

    bool use_hand_order = false;   // 만족 쌍이 없으면 제약 해제
    for (size_t i = 0; i < n_R && !use_hand_order; i++) {
        for (size_t j = 0; j < n_L && !use_hand_order; j++) use_hand_order = pair_order(i, j);
    }

    const int keep_idx_R = scores_R[0].keep_idx;   // 반대손 무관
    const int keep_idx_L = scores_L[0].keep_idx;

    // 탐색 집합을 좁은 것부터 (같은 악기 반복 유지 우선): {kR}×{kL} → {kR}×전체 → 전체×{kL} → 전체×전체. 최선 폭 > 0 인 첫 집합에서 멈춤
    struct Range { size_t lo, hi; };   // [lo, hi)
    const Range all_R{0, n_R}, all_L{0, n_L};
    const Range keep_R{(size_t)std::max(keep_idx_R, 0), (size_t)std::max(keep_idx_R, 0) + 1};
    const Range keep_L{(size_t)std::max(keep_idx_L, 0), (size_t)std::max(keep_idx_L, 0) + 1};
    std::vector<std::pair<Range, Range>> passes;
    if (keep_idx_R >= 0 && keep_idx_L >= 0) passes.push_back({keep_R, keep_L});
    if (keep_idx_R >= 0)                    passes.push_back({keep_R, all_L});
    if (keep_idx_L >= 0)                    passes.push_back({all_R, keep_L});
    passes.push_back({all_R, all_L});

    int best_i = -1, best_j = -1, best_width = -1;
    for (const auto& [range_R, range_L] : passes) {
        best_i = -1; best_j = -1; best_width = -1;
        for (size_t i = range_R.lo; i < range_R.hi; i++) {
            for (size_t j = range_L.lo; j < range_L.hi; j++) {
                if (use_hand_order && !pair_order(i, j)) continue;
                const int width = pair_width(i, j);
                if (width > best_width) { best_width = width; best_i = (int)i; best_j = (int)j; }   // 동률: 낮은 i, 낮은 j
            }
        }
        if (best_width > 0) break;
    }

    if (best_width <= 0) {
        // 반대손과 양립하는 허리각이 없음: 팔별 단독 폭 최대로 폴백 (이후 compute_waist_range가 오류 처리)
        auto solo_best = [&](Arm arm, const std::vector<std::array<double, 3>>& candidates, int instrument) {
            const double wrist = get_wrist_angle(instrument);
            int best = 0, best_count = -1;
            for (size_t i = 0; i < candidates.size(); i++) {
                const int count = (int)get_feasible_waist_range(arm, candidates[i], wrist).count();
                if (count > best_count) { best_count = count; best = (int)i; }
            }
            return best;
        };
        best_i = solo_best(Arm::RIGHT, *candidates_R, seg_R.end_instrument);
        best_j = solo_best(Arm::LEFT,  *candidates_L, seg_L.end_instrument);
    }

    if (verbose) {
        std::cerr << "[BaseMotionGenerator] select pair t_R=" << seg_R.end_time << " t_L=" << seg_L.end_time
                  << " R instrument=" << seg_R.end_instrument << " candidate=" << best_i << "/" << n_R
                  << " L instrument=" << seg_L.end_instrument << " candidate=" << best_j << "/" << n_L
                  << " width=" << std::max(best_width, 0) * 0.1 << "deg"
                  << " keep_idx=" << keep_idx_R << "," << keep_idx_L
                  << " hand_order=" << use_hand_order << "\n";
    }

    return {(*candidates_R)[best_i], (*candidates_L)[best_j]};
}

std::pair<std::array<double, 3>, std::array<double, 3>> BaseMotionGenerator::get_end_positions(const MotionSegment& seg_R, const MotionSegment& seg_L,
                                                                                    bool need_select_R, bool need_select_L, bool verbose) 
{
    // 반환값
    std::array<double, 3> end_pos_R = seg_R.end_position;
    std::array<double, 3> end_pos_L = seg_L.end_position;

    // 후보점 position 로드
    const std::vector<std::array<double, 3>>* candidates_R = get_candidate_positions(Arm::RIGHT, seg_R.end_instrument);
    const std::vector<std::array<double, 3>>* candidates_L = get_candidate_positions(Arm::LEFT,  seg_L.end_instrument);

    // 후보점 개수
    const size_t num_cand_R = candidates_R ? candidates_R->size() : 0;
    const size_t num_cand_L = candidates_L ? candidates_L->size() : 0;

    // 선정 여부 복사
    bool pending_R = need_select_R;
    bool pending_L = need_select_L;

    // 후보가 0~1개인 팔 예외처리
    // todo : fallback 좌표를 선정. 현재는 center임. 근데 bell류는 중심 치면 안됨.
    MotionSegment fixed_R = seg_R, fixed_L = seg_L;
    if (pending_R && num_cand_R <= 1) { end_pos_R = fixed_R.end_position = get_fallback_position(Arm::RIGHT, seg_R.end_instrument); pending_R = false; }
    if (pending_L && num_cand_L <= 1) { end_pos_L = fixed_L.end_position = get_fallback_position(Arm::LEFT, seg_L.end_instrument); pending_L = false; }

    // 한 손만 선정
    if (pending_R != pending_L) {
        if (pending_R) end_pos_R = select_hit_position(Arm::RIGHT, seg_R, fixed_L, verbose);
        else           end_pos_L = select_hit_position(Arm::LEFT,  seg_L, fixed_R, verbose);
        return {end_pos_R, end_pos_L};
    }
    if (!pending_R) return {end_pos_R, end_pos_L};


    return {end_pos_R, end_pos_L};
}

std::array<double, 3> BaseMotionGenerator::get_fallback_position(Arm arm, int instrument){

}

// =====================================

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
        auto [seg_R, seg_L] = get_motion_segments(std::vector<DrumEvent>(rds.begin() + i, rds.end()), tmp_context_R, tmp_context_L);

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
        double the0 = -0.5 * M_PI + M_PI / 1800.0 * i;  //theta_0 : -90 deg ~ 90 deg

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