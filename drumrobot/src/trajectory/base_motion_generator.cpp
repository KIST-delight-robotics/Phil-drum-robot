#include "trajectory/base_motion_generator.hpp"

BaseMotionGenerator::BaseMotionGenerator() {

}

BaseMotionGenerator::~BaseMotionGenerator() {

}

void BaseMotionGenerator::initialize(const std::map<int, InstrumentCoordinate>& coordinates) {
 
    solver.initialize();
    drum_coordinates = coordinates;
}

BaseMotionPoint BaseMotionGenerator::reset() {
    right_context = MotionContext{};
    left_context = MotionContext{};

    BaseMotionPoint point;
    note_to_target(1, Arm::RIGHT, point.right_position, point.right_wrist); // 스네어
    note_to_target(1, Arm::LEFT, point.left_position, point.left_wrist);

    point.waist = 0.0;  // TODO: 임시 허리각

    return point;
}

std::queue<BaseMotionPoint> BaseMotionGenerator::generate_motion(const std::vector<DrumEvent>& rds, int num_point) {
    std::queue<BaseMotionPoint> out;
 
    if (rds.size() < 2 || num_point <= 0) {
        return out;
    }

    MotionSegment seg_R = get_motion_segment(rds, Arm::RIGHT);
    MotionSegment seg_L = get_motion_segment(rds, Arm::LEFT);

    right_context = seg_R.next_context; // context update
    left_context = seg_L.next_context;

    for (int i = 0; i < num_point; i++) {
        BaseMotionPoint point;

        double t_R = i * ROBOT::DT_SECOND + seg_R.t0 - seg_R.start_time;
        double t_L = i * ROBOT::DT_SECOND + seg_L.t0 - seg_L.start_time;

        double s_R = time_scaling(0.0, seg_R.end_time - seg_R.start_time, t_R);
        double s_L = time_scaling(0.0, seg_L.end_time - seg_L.start_time, t_L);
        
        point.right_position = make_path(seg_R.start_position, seg_R.end_position, s_R);
        point.left_position = make_path(seg_L.start_position, seg_L.end_position, s_L);

        point.right_wrist = s_R * (seg_R.end_wrist_angle - seg_R.start_wrist_angle) + seg_R.start_wrist_angle;
        point.left_wrist = s_L * (seg_L.end_wrist_angle - seg_L.start_wrist_angle) + seg_L.start_wrist_angle;

        point.waist = 0.0;  // TODO: 임시 허리각

        out.push(point);
    }

    return out;
}

BaseMotionGenerator::MotionSegment BaseMotionGenerator::get_motion_segment(const std::vector<DrumEvent>& rds, Arm arm) {
    MotionSegment seg;

    seg.t0 = rds[0].t;
    seg.t1 = rds[1].t;

    // 팔에 따라 참조할 context / note 선택
    MotionContext& context = (arm == Arm::RIGHT) ? right_context : left_context;
    auto note_of = [&](int i) {
        return (arm == Arm::RIGHT) ? rds[i].note_num_R : rds[i].note_num_L;
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
            note_hit = note_of(i);  // TODO: 오픈 하이햇 처리 해야함
        }
    }

    // ===== 악기 & 시간 =====
    MotionContext next_context;

    if (note_of(0) == 0) {
        if (context.state == State::REST_TO_HIT || context.state == State::HIT_TO_HIT) {
            // 기존의 이동하던 궤적을 이어서 이동
            note_to_target(context.last_instrument, arm,
                           seg.start_position, seg.start_wrist_angle);
            note_to_target(note_hit, arm,
                           seg.end_position, seg.end_wrist_angle);

            seg.start_time = context.last_t;
            seg.end_time   = t_hit;

            next_context = context;
        } else {
            if (is_hit) {
                // 휴식 중 다음 타격 찾음
                note_to_target(context.last_instrument, arm,
                               seg.start_position, seg.start_wrist_angle);
                note_to_target(note_hit, arm,
                               seg.end_position, seg.end_wrist_angle);

                seg.start_time = rds[0].t;
                seg.end_time   = t_hit;

                next_context.state          = State::REST_TO_HIT;
                next_context.last_t         = rds[0].t;
                next_context.last_instrument = context.last_instrument;

            } else {
                // 휴식 중 다음 타격 없음 (현재 위치 유지)
                note_to_target(context.last_instrument, arm,
                               seg.start_position, seg.start_wrist_angle);
                seg.end_position    = seg.start_position;
                seg.end_wrist_angle = seg.start_wrist_angle;

                seg.start_time = rds[0].t;
                seg.end_time   = rds[1].t;

                next_context.state          = State::REST_TO_REST;
                next_context.last_t         = rds[0].t;
                next_context.last_instrument = context.last_instrument;
            }
        }
    } else {
        if (is_hit) {
            // 타격 후 다음 타격 찾음
            note_to_target(note_of(0), arm,
                           seg.start_position, seg.start_wrist_angle);
            note_to_target(note_hit, arm,
                           seg.end_position, seg.end_wrist_angle);

            seg.start_time = rds[0].t;
            seg.end_time   = t_hit;

            next_context.state          = State::HIT_TO_HIT;
            next_context.last_t         = rds[0].t;
            next_context.last_instrument = note_of(0);
        } else {
            // 타격 후 다음 타격 없음 (현재 위치 유지)
            note_to_target(note_of(0), arm,
                           seg.start_position, seg.start_wrist_angle);
            seg.end_position    = seg.start_position;
            seg.end_wrist_angle = seg.start_wrist_angle;

            seg.start_time = rds[0].t;
            seg.end_time   = rds[1].t;

            next_context.state          = State::HIT_TO_REST;
            next_context.last_t         = rds[0].t;
            next_context.last_instrument = note_of(0);
        }
    }

    seg.next_context = next_context;

    return seg;
}

void BaseMotionGenerator::note_to_target(int note_num, Arm arm, std::array<double, 3>& out_position, double& out_wrist_angle_deg) {
    auto it = drum_coordinates.find(note_num);
    if (it == drum_coordinates.end()) {
        std::cerr << "[BaseMotionGenerator] note_to_target: invalid note number "
                  << note_num << "\n";
        out_position        = {0.0, 0.0, 0.0};
        out_wrist_angle_deg = 0.0;
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
        double h1 = 0.0, h2 = 0.0;  // TODO: 조절해야 함
        
        std::array<double, 3> pm1;
        std::array<double, 3> pm2;
        for (int i = 0; i < 3; i++) {
            pm1[i] = (2.0/3.0) * pi[i] + (1.0/3.0) * pf[i];
            pm2[i] = (1.0/3.0) * pi[i] + (2.0/3.0) * pf[i];
        }
        pm1[2] += h1;
        pm2[2] += h2;

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