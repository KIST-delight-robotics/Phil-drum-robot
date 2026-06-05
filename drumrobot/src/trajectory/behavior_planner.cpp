#include "trajectory/behavior_planner.hpp"

// 관절 ID 상수 (motors.json 참조)
namespace JointID {
    constexpr int WAIST            = 0;
    constexpr int R_SHOULDER_1     = 1;
    constexpr int L_SHOULDER_1     = 2;
    constexpr int R_SHOULDER_2     = 3;
    constexpr int R_ELBOW          = 4;
    constexpr int L_SHOULDER_2     = 5;
    constexpr int L_ELBOW          = 6;
    constexpr int R_WRIST          = 7;
    constexpr int L_WRIST          = 8;
    constexpr int R_PEDAL          = 9;
    constexpr int L_PEDAL          = 10;
    constexpr int HEAD_YAW         = 11;
    constexpr int HEAD_PITCH       = 12;
}

BehaviorPlanner::BehaviorPlanner(AppContext &ctxRef, Robot &robotRef)
    : ctx(ctxRef), robot(robotRef) {
    // 초기 자세를 last_q_target으로 설정 (모터의 initial_joint_angle 사용)
    last_q_target.resize(ROBOT::NUM_JOINT, 0.0);
    for (const auto &[id, motor] : robot.motors) {
        if (id < ROBOT::NUM_JOINT) {
            last_q_target[id] = motor->initial_joint_angle;
        }
    }
}

BehaviorPlanner::~BehaviorPlanner() {

}

std::vector<MotionPrimitive> BehaviorPlanner::generate_motion_sequence(const ParsedCommand& parsed) {
    std::vector<MotionPrimitive> sequence;

    if (!parsed.valid) {
        std::cerr << "[BehaviorPlanner] Invalid command\n";
        return sequence;
    }

    Opcode opcode = parsed.opcode;

    // ===== send_active 전 =====
    // 시작/종료 명령만 처리
    if (!ctx.send_active.load()) {
        if (opcode == Opcode::START) {
            return handle_start();
        } else if (opcode == Opcode::QUIT) {
            ctx.robot_state = RobotState::ShuttingDown;
            return sequence;
        } else {
            std::cerr << "[BehaviorPlanner] 수행할 수 없는 명령 (send_active=false): opcode="
                      << static_cast<int>(opcode) << "\n";
            return sequence;
        }
    }

    // ===== send_active 후 =====
    switch (opcode) {
        case Opcode::STATE: {
            handle_state(parsed.args);
            return sequence;
        }
        case Opcode::LOOK:    return handle_look(parsed.args);
        case Opcode::GESTURE: return handle_gesture(parsed.args);
        case Opcode::MOVE:    return handle_move(parsed.args);
        case Opcode::POSE:    return handle_pose(parsed.args);
        case Opcode::HIT:     return handle_hit(parsed.args);
        case Opcode::PLAY:    return handle_play(parsed.args);
        case Opcode::START:
            std::cerr << "[BehaviorPlanner] 이미 시작된 상태\n";
            return sequence;
        case Opcode::QUIT: {
            // shutdown 포즈로 이동 후 종료 플래그 세팅
            auto it = poses.find("shutdown");
            if (it != poses.end()) {
                sequence.push_back(make_translate(it->second, DEFAULT_MOVE_TIME));
                last_q_target = it->second;
            }
            ctx.robot_state = RobotState::ShuttingDown;
            return sequence;
        }
        default:
            std::cerr << "[BehaviorPlanner] Unknown opcode\n";
            return sequence;
    }
}

void BehaviorPlanner::init_poses_from_json() {
    using json = nlohmann::json;

    std::ifstream f("drumrobot/config/robot_poses.json");
    if (!f.is_open()) {
        std::cerr << "[BehaviorPlanner] Failed to open config/robot_poses.json\n";
        return;
    }
    json config = json::parse(f);

    for (auto &[name, angles] : config["poses"].items()) {
        for (auto &a : angles) {
            poses[name].push_back(a.get<double>() * M_PI / 180.0);
        }
    }
}

// =============================================================
// Opcode별 핸들러
// =============================================================

// START: home 포즈로 이동
std::vector<MotionPrimitive> BehaviorPlanner::handle_start() {
    std::vector<MotionPrimitive> sequence;

    auto it = poses.find("home");
    if (it == poses.end()) {
        std::cerr << "[BehaviorPlanner] 'home' pose not found in robot_poses.json\n";
        return sequence;
    }

    sequence.push_back(make_translate(it->second, DEFAULT_MOVE_TIME));
    last_q_target = it->second;

    return sequence;
}

// STATE: state 변경
void BehaviorPlanner::handle_state(const std::vector<std::string>& args) {
    const std::string& state = args[0];

    if (state == "idle" && ctx.robot_state.load() == RobotState::Init) {
        ctx.robot_state = RobotState::Idle;
    } else {
        std::cerr << "[BehaviorPlanner] Unknown state argument: " << state << "\n";
    }
}

// LOOK pan tilt : 머리 yaw, pitch 제어
std::vector<MotionPrimitive> BehaviorPlanner::handle_look(const std::vector<std::string>& args) {
    std::vector<MotionPrimitive> sequence;
    if (ctx.robot_state.load() != RobotState::Idle) {
        std::cerr << "[BehaviorPlanner] LOOK rejected: only allowed in Idle\n";
        return sequence;
    }

    try {
        double pan_deg  = std::stod(args[0]);
        double tilt_deg = std::stod(args[1]);

        // 마지막 목표를 복사해서 head 관절만 갱신
        std::vector<double> q_target = last_q_target;
        q_target[JointID::HEAD_YAW]   = deg_to_rad(pan_deg);
        q_target[JointID::HEAD_PITCH] = deg_to_rad(tilt_deg);

        sequence.push_back(make_translate(q_target, LOOK_MOVE_TIME));
        last_q_target = q_target;
    } catch (const std::exception &e) {
        std::cerr << "[BehaviorPlanner] LOOK parsing error: " << e.what() << "\n";
    }

    return sequence;
}

// GESTURE type : 미리 정의된 제스처 시퀀스
std::vector<MotionPrimitive> BehaviorPlanner::handle_gesture(const std::vector<std::string>& args) {
    std::vector<MotionPrimitive> sequence;
    if (ctx.robot_state.load() != RobotState::Idle) {
        std::cerr << "[BehaviorPlanner] GESTURE rejected: only allowed in Idle\n";
        return sequence;
    }

    const std::string& type = args[0];

    if (type == "nod") {
        // 끄덕임: 아래 → 위 → 정면
        std::vector<double> q;
        q = last_q_target; q[JointID::HEAD_PITCH] = deg_to_rad(20.0);
        sequence.push_back(make_translate(q, GESTURE_MOVE_TIME));
        q[JointID::HEAD_PITCH] = deg_to_rad(-20.0);
        sequence.push_back(make_translate(q, GESTURE_MOVE_TIME));
        q[JointID::HEAD_PITCH] = 0.0;
        sequence.push_back(make_translate(q, GESTURE_MOVE_TIME));
        last_q_target = q;
    }
    else if (type == "shake") {
        // 도리도리: 좌 → 우 → 정면
        std::vector<double> q;
        q = last_q_target; q[JointID::HEAD_YAW] = deg_to_rad(30.0);
        sequence.push_back(make_translate(q, GESTURE_MOVE_TIME));
        q[JointID::HEAD_YAW] = deg_to_rad(-30.0);
        sequence.push_back(make_translate(q, GESTURE_MOVE_TIME));
        q[JointID::HEAD_YAW] = 0.0;
        sequence.push_back(make_translate(q, GESTURE_MOVE_TIME));
        last_q_target = q;
    }
    else if (type == "wave" || type == "hi") {
        // 인사: 오른팔 들기 + 손목 흔들기
        // 1) 오른팔 인사 자세
        std::vector<double> q = last_q_target;
        q[JointID::R_SHOULDER_1] = deg_to_rad(45.0);
        q[JointID::R_SHOULDER_2] = deg_to_rad(45.0);
        q[JointID::R_ELBOW]      = deg_to_rad(90.0);
        q[JointID::R_WRIST]      = 0.0;
        q[JointID::HEAD_YAW]     = deg_to_rad(-20.0);
        q[JointID::HEAD_PITCH]   = deg_to_rad(-5.0);
        sequence.push_back(make_translate(q, DEFAULT_MOVE_TIME));

        // 2) 손목 좌우 흔들기 3회
        for (int i = 0; i < 3; i++) {
            q[JointID::R_WRIST] = deg_to_rad(25.0);
            sequence.push_back(make_translate(q, 0.4));
            q[JointID::R_WRIST] = deg_to_rad(-25.0);
            sequence.push_back(make_translate(q, 0.4));
        }
        // 복귀
        q[JointID::R_WRIST] = 0.0;
        sequence.push_back(make_translate(q, 0.4));
        last_q_target = q;
    }
    else if (type == "hurray" || type == "happy") {
        // 환호: 양팔 들기
        std::vector<double> q = last_q_target;
        q[JointID::R_SHOULDER_1] = deg_to_rad(60.0);
        q[JointID::L_SHOULDER_1] = deg_to_rad(120.0);
        q[JointID::R_SHOULDER_2] = deg_to_rad(65.0);
        q[JointID::L_SHOULDER_2] = deg_to_rad(65.0);
        q[JointID::R_ELBOW]      = deg_to_rad(95.0);
        q[JointID::L_ELBOW]      = deg_to_rad(95.0);
        q[JointID::R_WRIST]      = 0.0;
        q[JointID::L_WRIST]      = 0.0;
        q[JointID::HEAD_PITCH]   = deg_to_rad(-15.0);
        sequence.push_back(make_translate(q, DEFAULT_MOVE_TIME));
        last_q_target = q;
    }
    else {
        std::cerr << "[BehaviorPlanner] Unknown gesture: " << type << "\n";
    }

    return sequence;
}

// MOVE motor_name angle_deg [move_time]
std::vector<MotionPrimitive> BehaviorPlanner::handle_move(const std::vector<std::string>& args) {   // TODO: 여러개의 관절 동시에 움직이기
    std::vector<MotionPrimitive> sequence;
    if (ctx.robot_state.load() != RobotState::Idle) {
        std::cerr << "[BehaviorPlanner] MOVE rejected: only allowed in Idle\n";
        return sequence;
    }

    const std::string& motor_name = args[0];
    int motor_id = find_motor_id(motor_name);
    if (motor_id < 0) {
        std::cerr << "[BehaviorPlanner] Unknown motor name: " << motor_name << "\n";
        return sequence;
    }

    try {
        double angle_deg = std::stod(args[1]);
        double move_time = (args.size() >= 3) ? std::stod(args[2]) : DEFAULT_MOVE_TIME;

        std::vector<double> q_target = last_q_target;
        q_target[motor_id] = deg_to_rad(angle_deg);

        sequence.push_back(make_translate(q_target, move_time));
        last_q_target = q_target;
    } catch (const std::exception &e) {
        std::cerr << "[BehaviorPlanner] MOVE parsing error: " << e.what() << "\n";
    }

    return sequence;
}

// POSE pose_name : 사전 정의 포즈로 이동
std::vector<MotionPrimitive> BehaviorPlanner::handle_pose(const std::vector<std::string>& args) {
    std::vector<MotionPrimitive> sequence;
    if (ctx.robot_state.load() != RobotState::Idle) {
        std::cerr << "[BehaviorPlanner] POSE rejected: only allowed in Idle\n";
        return sequence;
    }

    const std::string& pose_name = args[0];

    auto it = poses.find(pose_name);
    if (it == poses.end()) {
        std::cerr << "[BehaviorPlanner] Unknown pose: " << pose_name << "\n";
        return sequence;
    }

    sequence.push_back(make_translate(it->second, DEFAULT_MOVE_TIME, TrajectoryProfile::TRAPEZOIDAL));
    last_q_target = it->second;

    // shutdown 포즈로 이동하는 경우 종료 플래그 세팅
    if (pose_name == "shutdown") {
        ctx.robot_state = RobotState::ShuttingDown;
    }

    return sequence;
}

// HIT target : 드럼 타격
std::vector<MotionPrimitive> BehaviorPlanner::handle_hit(const std::vector<std::string>& args) {
    std::vector<MotionPrimitive> sequence;
    if (ctx.robot_state.load() != RobotState::Idle) {
        std::cerr << "[BehaviorPlanner] HIT rejected: only allowed in Idle\n";
        return sequence;
    }

    MotionPrimitive start;
    start.type = MotionType::DRUM;
    start.flag = PlayFlag::START;

    MotionPrimitive end;
    end.type = MotionType::DRUM;
    end.flag = PlayFlag::END;

    const std::string& target = args[0];

    // TODO: 악기 이름과 번호를 어디선가 정의되어 공유하면 좋을 듯
    if (target == "snare") {
        sequence.push_back(start);
        sequence.push_back(make_drum_hit(DEFAULT_HIT_TIME, 1, false, false));
        sequence.push_back(end);
    } else if (target == "floor") {
        sequence.push_back(start);
        sequence.push_back(make_drum_hit(DEFAULT_HIT_TIME, 2, false, false));
        sequence.push_back(end);
    } else if (target == "mid") {
        sequence.push_back(start);
        sequence.push_back(make_drum_hit(DEFAULT_HIT_TIME, 3, false, false));
        sequence.push_back(end);
    } else if (target == "top") {
        sequence.push_back(start);
        sequence.push_back(make_drum_hit(DEFAULT_HIT_TIME, 4, false, false));
        sequence.push_back(end);
    } else if (target == "closed") {    // closed hi-hat
        sequence.push_back(start);
        sequence.push_back(make_drum_hit(DEFAULT_HIT_TIME, 5, false, true));
        sequence.push_back(end);
    } else if (target == "open") {      // open hi-hat
        sequence.push_back(start);
        sequence.push_back(make_drum_hit(DEFAULT_HIT_TIME, 5, false, false));
        sequence.push_back(end);
    } else if (target == "ride") {
        sequence.push_back(start);
        sequence.push_back(make_drum_hit(DEFAULT_HIT_TIME, 6, false, false));
        sequence.push_back(end);
    } else if (target == "right") {     // right crash
        sequence.push_back(start);
        sequence.push_back(make_drum_hit(DEFAULT_HIT_TIME, 7, false, false));
        sequence.push_back(end);
    } else if (target == "left") {      // left crash
        sequence.push_back(start);
        sequence.push_back(make_drum_hit(DEFAULT_HIT_TIME, 8, false, false));
        sequence.push_back(end);
    } else if (target == "bass") {
        sequence.push_back(start);
        sequence.push_back(make_drum_hit(DEFAULT_HIT_TIME, 0, true, false));
        sequence.push_back(end);
    } else {
        std::cerr << "[BehaviorPlanner] Unknown target instrument: " << target << "\n";
        return sequence;
    }

    return sequence;
}

// PLAY score_name : 드럼 연주
std::vector<MotionPrimitive> BehaviorPlanner::handle_play(const std::vector<std::string>& args) {
    std::vector<MotionPrimitive> sequence;
    if (ctx.robot_state.load() != RobotState::Idle) {
        std::cerr << "[BehaviorPlanner] PLAY rejected: only allowed in Idle\n";
        return sequence;
    }

    ctx.robot_state = RobotState::Playing;
    const std::string& score_name = args[0];

    std::ifstream inputFile;
    inputFile.open(score_name); // 파일 열기

    if (!inputFile.is_open()) {
        return sequence;
    }

    string row;
    while (getline(inputFile, row)) {
        istringstream iss(row);
        string item;
        vector<string> items;
        
        while (getline(iss, item, '\t')) {
            item = trim_whitespace(item);
            items.push_back(item);
        }

        if (items[0] == "bpm") {

        } else if (items[0] == "end") {

        } else {
            MotionPrimitive motion;
            motion.type = MotionType::DRUM;
            motion.robotic_drum_score.push_back(make_drum_event(items));

            sequence.push_back(motion);
        }
    }
    inputFile.close();

    return sequence;
}

// =============================================================
// 헬퍼
// =============================================================

MotionPrimitive BehaviorPlanner::make_translate(const std::vector<double>& q_target,
                                                double t_total,
                                                TrajectoryProfile profile) {
    MotionPrimitive motion;
    motion.type     = MotionType::TRANSLATE;
    motion.space    = TrajectorySpace::JOINT;
    motion.profile  = profile;
    motion.q_target = q_target;
    motion.t_total  = t_total;
    return motion;
}

MotionPrimitive BehaviorPlanner::make_drum_hit(double t, int note_num, bool is_kick, bool is_closed_hihat) {
    MotionPrimitive motion;
    motion.type     = MotionType::DRUM;

    DrumEvent Dummy;
    motion.robotic_drum_score.push_back(Dummy);     // rds[0]

    DrumEvent event;
    event.bar = 1;
    event.t = t;
    if (note_num == 1 || note_num == 4 || note_num == 5 || note_num == 8) {
        event.note_num_L = note_num;
        event.velocity_L = 5;
    } else {
        event.note_num_R = note_num;
        event.velocity_R = 5;
    }
    event.is_kick = is_kick;
    event.is_closed_hihat = is_closed_hihat;

    motion.robotic_drum_score.push_back(event);     // rds[1]

    return motion;
}

std::string BehaviorPlanner::trim_whitespace(const std::string &str) {
    size_t first = str.find_first_not_of(" \t");
    if (std::string::npos == first) {
        return str;
    }
    size_t last = str.find_last_not_of(" \t");
    return str.substr(first, (last - first + 1));
}

DrumEvent BehaviorPlanner::make_drum_event(const std::vector<std::string>& items) {
    DrumEvent event;

    event.bar             = stoi(items[0]);
    event.beat            = stod(items[1]);
    event.note_num_R      = stoi(items[2]);
    event.note_num_L      = stoi(items[3]);
    event.velocity_R      = stoi(items[4]);
    event.velocity_L      = stoi(items[5]);
    event.is_kick         = (stoi(items[6]) == 1);
    event.is_closed_hihat = (stoi(items[7]) == 1);
    event.t               = event.beat;     // TODO: bpm 곱해서 누적하기

    return event;
}

int BehaviorPlanner::find_motor_id(const std::string& motor_name) const {
    for (const auto &[id, motor] : robot.motors) {
        if (motor->name == motor_name) return id;
    }
    return -1;
}