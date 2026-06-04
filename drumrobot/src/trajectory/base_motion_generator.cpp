#include "trajectory/base_motion_generator.hpp"

BaseMotionGenerator::BaseMotionGenerator() {

}

BaseMotionGenerator::~BaseMotionGenerator() {

}

void BaseMotionGenerator::initialize(const std::map<int, InstrumentCoordinate>& coordinates) {
 
    solver.initialize();
    drum_coordinates = coordinates;
}

std::queue<BaseMotionPoint> BaseMotionGenerator::generate_motion(const std::vector<DrumEvent>& rds, int num_point) {
    std::queue<BaseMotionPoint> out;
 
    if (rds.size() < 2 || num_point <= 0) {
        return out;
    }

    MotionSegment seg = get_motion_segment(rds);

    right_context = seg.right.next_context; // context update
    left_context = seg.left.next_context;

    for (int i = 0; i < num_point; i++) {
        BaseMotionPoint point;

        double t_R = i * ROBOT::DT_SECOND + seg.t0 - seg.right.start_time;
        double t_L = i * ROBOT::DT_SECOND + seg.t0 - seg.left.start_time;

        double s_R = time_scaling(0.0, seg.right.end_time - seg.right.start_time, t_R);
        double s_L = time_scaling(0.0, seg.left.end_time - seg.left.start_time, t_L);
        
        point.right_position = make_path(seg.right.start_position, seg.right.end_position, s_R);
        point.left_position = make_path(seg.left.start_position, seg.left.end_position, s_L);

        point.right_wrist = s_R * (seg.right.end_wrist_angle - seg.right.start_wrist_angle) + seg.right.start_wrist_angle;
        point.left_wrist = s_L * (seg.left.end_wrist_angle - seg.left.start_wrist_angle) + seg.left.start_wrist_angle;

        // TODO: 허리각 결정해줘야 함

        out.push(point);
    }

    return out;
}

BaseMotionGenerator::MotionSegment BaseMotionGenerator::get_motion_segment(const std::vector<DrumEvent>& rds) {
    MotionSegment seg;
    
    seg.t0 = rds[0].t;
    seg.t1 = rds[1].t;

    // ===== 타격 감지 =====
    const double e = 0.00001; 
    int rds_size = rds.size();
    
    bool is_hit_R = false, is_hit_L = false;
    double t_hit_R = 0.0, t_hit_L = 0.0;
    int note_hit_R = 0, note_hit_L = 0;

    // R
    for (int i = 1; i < rds_size; i++) {
        if (round(10000 * (HIT_DETECTION_THRESHOLD + e)) < round(10000 * (rds[i].t - rds[0].t))) {
            break;
        }

        if (rds[i].note_num_R != 0) {
            is_hit_R = true;
            t_hit_R = rds[i].t;
            note_hit_R = rds[i].note_num_R; // TODO: 오픈 하이햇 처리 해야함
        }
    }

    // L
    for (int i = 1; i < rds_size; i++) {
        if (round(10000 * (HIT_DETECTION_THRESHOLD + e)) < round(10000 * (rds[i].t - rds[0].t))) {
            break;
        }

        if (rds[i].note_num_L != 0) {
            is_hit_L = true;
            t_hit_L = rds[i].t;
            note_hit_L = rds[i].note_num_L; // TODO: 오픈 하이햇 처리 해야함
        }
    }

    // ===== 악기 & 시간 =====
    // R
    MotionContext next_context_R;
 
    if (rds[0].note_num_R == 0) {
        if (right_context.state == State::REST_TO_HIT || right_context.state == State::HIT_TO_HIT) {
            // 기존의 이동하던 궤적을 이어서 이동
            next_context_R.state = right_context.state;
            next_context_R.last_t = right_context.last_t;
            next_context_R.last_instrument = right_context.last_instrument;
 
            note_to_target(right_context.last_instrument, Arm::RIGHT,
                           seg.right.start_position, seg.right.start_wrist_angle);
            note_to_target(note_hit_R, Arm::RIGHT,
                           seg.right.end_position, seg.right.end_wrist_angle);
 
            seg.right.start_time = right_context.last_t;
            seg.right.end_time = t_hit_R;
        } else {
            if (is_hit_R) {
                // 휴식 중 다음 타격 찾음
                next_context_R.state = State::REST_TO_HIT;
                next_context_R.last_t = rds[0].t;
                next_context_R.last_instrument = right_context.last_instrument;
 
                note_to_target(right_context.last_instrument, Arm::RIGHT,
                               seg.right.start_position, seg.right.start_wrist_angle);
                note_to_target(note_hit_R, Arm::RIGHT,
                               seg.right.end_position, seg.right.end_wrist_angle);
 
                seg.right.start_time = rds[0].t;
                seg.right.end_time = t_hit_R;
            } else {
                // 휴식 중 다음 타격 없음 (현재 위치 유지)
                next_context_R.state = State::REST_TO_REST;
                next_context_R.last_t = rds[0].t;
                next_context_R.last_instrument = right_context.last_instrument;
 
                note_to_target(right_context.last_instrument, Arm::RIGHT,
                               seg.right.start_position, seg.right.start_wrist_angle);
                seg.right.end_position    = seg.right.start_position;
                seg.right.end_wrist_angle = seg.right.start_wrist_angle;
 
                seg.right.start_time = rds[0].t;
                seg.right.end_time = rds[1].t;
            }
        }
    } else {
        if (is_hit_R) {
            // 타격 후 다음 타격 찾음
            next_context_R.state = State::HIT_TO_HIT;
            next_context_R.last_t = rds[0].t;
            next_context_R.last_instrument = rds[0].note_num_R;
 
            note_to_target(rds[0].note_num_R, Arm::RIGHT,
                           seg.right.start_position, seg.right.start_wrist_angle);
            note_to_target(note_hit_R, Arm::RIGHT,
                           seg.right.end_position, seg.right.end_wrist_angle);
 
            seg.right.start_time = rds[0].t;
            seg.right.end_time = t_hit_R;
        } else {
            // 타격 후 다음 타격 없음 (현재 위치 유지)
            next_context_R.state = State::HIT_TO_REST;
            next_context_R.last_t = rds[0].t;
            next_context_R.last_instrument = rds[0].note_num_R;
 
            note_to_target(rds[0].note_num_R, Arm::RIGHT,
                           seg.right.start_position, seg.right.start_wrist_angle);
            seg.right.end_position    = seg.right.start_position;
            seg.right.end_wrist_angle = seg.right.start_wrist_angle;
 
            seg.right.start_time = rds[0].t;
            seg.right.end_time = rds[1].t;
        }
    }
 
    seg.right.next_context = next_context_R;
 
    // L
    MotionContext next_context_L;
 
    if (rds[0].note_num_L == 0) {
        if (left_context.state == State::REST_TO_HIT || left_context.state == State::HIT_TO_HIT) {
            // 기존의 이동하던 궤적을 이어서 이동
            next_context_L.state = left_context.state;
            next_context_L.last_t = left_context.last_t;
            next_context_L.last_instrument = left_context.last_instrument;
 
            note_to_target(left_context.last_instrument, Arm::LEFT,
                           seg.left.start_position, seg.left.start_wrist_angle);
            note_to_target(note_hit_L, Arm::LEFT,
                           seg.left.end_position, seg.left.end_wrist_angle);
 
            seg.left.start_time = left_context.last_t;
            seg.left.end_time = t_hit_L;
        } else {
            if (is_hit_L) {
                // 휴식 중 다음 타격 찾음
                next_context_L.state = State::REST_TO_HIT;
                next_context_L.last_t = rds[0].t;
                next_context_L.last_instrument = left_context.last_instrument;
 
                note_to_target(left_context.last_instrument, Arm::LEFT,
                               seg.left.start_position, seg.left.start_wrist_angle);
                note_to_target(note_hit_L, Arm::LEFT,
                               seg.left.end_position, seg.left.end_wrist_angle);
 
                seg.left.start_time = rds[0].t;
                seg.left.end_time = t_hit_L;
            } else {
                // 휴식 중 다음 타격 없음 (현재 위치 유지)
                next_context_L.state = State::REST_TO_REST;
                next_context_L.last_t = rds[0].t;
                next_context_L.last_instrument = left_context.last_instrument;
 
                note_to_target(left_context.last_instrument, Arm::LEFT,
                               seg.left.start_position, seg.left.start_wrist_angle);
                seg.left.end_position    = seg.left.start_position;
                seg.left.end_wrist_angle = seg.left.start_wrist_angle;
 
                seg.left.start_time = rds[0].t;
                seg.left.end_time = rds[1].t;
            }
        }
    } else {
        if (is_hit_L) {
            // 타격 후 다음 타격 찾음
            next_context_L.state = State::HIT_TO_HIT;
            next_context_L.last_t = rds[0].t;
            next_context_L.last_instrument = rds[0].note_num_L;
 
            note_to_target(rds[0].note_num_L, Arm::LEFT,
                           seg.left.start_position, seg.left.start_wrist_angle);
            note_to_target(note_hit_L, Arm::LEFT,
                           seg.left.end_position, seg.left.end_wrist_angle);
 
            seg.left.start_time = rds[0].t;
            seg.left.end_time = t_hit_L;
        } else {
            // 타격 후 다음 타격 없음 (현재 위치 유지)
            next_context_L.state = State::HIT_TO_REST;
            next_context_L.last_t = rds[0].t;
            next_context_L.last_instrument = rds[0].note_num_L;
 
            note_to_target(rds[0].note_num_L, Arm::LEFT,
                           seg.left.start_position, seg.left.start_wrist_angle);
            seg.left.end_position    = seg.left.start_position;
            seg.left.end_wrist_angle = seg.left.start_wrist_angle;
 
            seg.left.start_time = rds[0].t;
            seg.left.end_time = rds[1].t;
        }
    }
 
    seg.left.next_context = next_context_L;
 
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
    if (arm == Arm::RIGHT) { out_position = coord.right_position; out_wrist_angle_deg = coord.right_wrist_angle_deg; }
    else                   { out_position = coord.left_position;  out_wrist_angle_deg = coord.left_wrist_angle_deg; }
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