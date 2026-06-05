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
        case Opcode::READY: {
            handle_ready();
            return sequence;
        }
        case Opcode::LOOK:    return handle_look(parsed.args);
        case Opcode::GESTURE: return handle_gesture(parsed.args);
        case Opcode::MOVE:    return handle_move(parsed.args);
        case Opcode::POSE:    return handle_pose(parsed.args);
        case Opcode::HIT:     return handle_hit(parsed.args);
        case Opcode::PLAY:    return handle_play(parsed.args);
        case Opcode::QUIT:    return handle_quit();
        case Opcode::START:
            std::cerr << "[BehaviorPlanner] 이미 시작된 상태\n";
            return sequence;
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

    std::cout   << "\n========================================\n"
                << " 모터 토크 ON\n"
                << " 1. 고정 키를 모두 제거하세요.\n"
                << " 2. 제거 후 'READY' 명령을 입력하세요.\n"
                << "========================================\n\n";
    ctx.robot_state = RobotState::Init;

    return sequence;
}

// READY: idle state로 변경
void BehaviorPlanner::handle_ready() {
    if (ctx.robot_state.load() == RobotState::Init) {
        ctx.robot_state = RobotState::Idle;
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

    const std::string& target = args[0];

    if (instrument_name_to_id.find(target) != instrument_name_to_id.end()) {
        MotionPrimitive start;
        start.type = MotionType::DRUM;
        start.flag = PlayFlag::START;
        sequence.push_back(start);

        int id = instrument_name_to_id.at(target);
        sequence.push_back(make_drum_hit(DEFAULT_HIT_TIME, id));

        MotionPrimitive end;
        end.type = MotionType::DRUM;
        end.flag = PlayFlag::END;
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
    std::string score_path = "drumrobot/data/scores/" + score_name + ".txt";
    inputFile.open(score_path); // 파일 열기

    if (!inputFile.is_open()) {
        std::cerr << "[BehaviorPlanner] PLAY: 악보 파일을 열 수 없습니다: " << score_path << "\n";
        return sequence;
    }

    std::vector<DrumEvent> rds;
    DrumEvent Dummy;
    rds.push_back(Dummy);   // rds[0]
    int start_idx = 0, end_idx = 0;

    double bpm = 100.0;
    double last_t = 0.0;

    MotionPrimitive start;
    start.type = MotionType::DRUM;
    start.flag = PlayFlag::START;
    sequence.push_back(start);

    std::string row;
    while (getline(inputFile, row)) {
        istringstream iss(row);
        std::string item;
        std::vector<std::string> items;
        
        while (getline(iss, item, '\t')) {
            item = trim_whitespace(item);
            items.push_back(item);
        }

        if (items[0] == "bpm") {
            bpm = stod(items[1]);
        } else if (items[0] == "end") {
            while (start_idx < end_idx) {
                sequence.push_back(make_drum_play(std::vector<DrumEvent>(rds.begin() + start_idx, rds.end())));
                start_idx++;
            }

            MotionPrimitive end;
            end.type = MotionType::DRUM;
            end.flag = PlayFlag::END;
            sequence.push_back(end);
        } else {
            rds.push_back(make_drum_event(items, bpm, last_t));

            end_idx++;
            last_t = rds[end_idx].t;

            // 2.4s : 100bpm 기준 한 마디 시간
            if ((rds[end_idx].t - rds[start_idx].t) * bpm / 100.0 >= 2.4) {
                sequence.push_back(make_drum_play(std::vector<DrumEvent>(rds.begin() + start_idx, rds.begin() + end_idx + 1))); // [start, end)
                start_idx++;
            }
        }
    }
    inputFile.close();

    return sequence;
}

std::vector<MotionPrimitive> BehaviorPlanner::handle_quit() {
    std::vector<MotionPrimitive> sequence;
    if (ctx.robot_state.load() != RobotState::Idle) {
        std::cerr << "[BehaviorPlanner] PLAY rejected: only allowed in Idle\n";
        return sequence;
    }

    // shutdown 포즈로 이동 후 종료 플래그 세팅
    auto it = poses.find("shutdown");
    if (it != poses.end()) {
        sequence.push_back(make_translate(it->second, DEFAULT_MOVE_TIME));
        last_q_target = it->second;
    }
    ctx.robot_state = RobotState::ShuttingDown;
    return sequence;
}

// =============================================================
// 헬퍼
// =============================================================

MotionPrimitive BehaviorPlanner::make_translate(const std::vector<double>& q_target, double t_total, TrajectoryProfile profile) {
    MotionPrimitive motion;
    motion.type     = MotionType::TRANSLATE;
    motion.space    = TrajectorySpace::JOINT;
    motion.profile  = profile;
    motion.q_target = q_target;
    motion.t_total  = t_total;
    return motion;
}

MotionPrimitive BehaviorPlanner::make_drum_hit(double t, int note_num) {
    MotionPrimitive motion;
    motion.type     = MotionType::DRUM;

    DrumEvent Dummy;
    motion.robotic_drum_score.push_back(Dummy);     // rds[0]

    DrumEvent event;
    event.bar = 1;
    event.t = t;
    if (note_num == 0) {
        event.is_kick = true;
    } else  if (note_num == 2 || note_num == 3 || note_num == 6 || note_num == 7) {
        event.note_num_R = note_num;
        event.velocity_R = 5;
    } else {
        event.note_num_L = note_num;
        event.velocity_L = 5;
        if (note_num == 9) event.is_closed_hihat = true;
    }

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

DrumEvent BehaviorPlanner::make_drum_event(const std::vector<std::string>& items, double bpm, double last_t) {
    DrumEvent event;
    event.bar             = stoi(items[0]);
    event.beat            = stod(items[1]);
    event.note_num_R      = stoi(items[2]);
    event.note_num_L      = stoi(items[3]);
    event.velocity_R      = stoi(items[4]);
    event.velocity_L      = stoi(items[5]);
    event.is_kick         = (stoi(items[6]) == 1);
    event.is_closed_hihat = (stoi(items[7]) == 1);
    event.t               = event.beat * 100.0 / bpm + last_t;
    return event;
}

MotionPrimitive BehaviorPlanner::make_drum_play(std::vector<DrumEvent> rds) {
    MotionPrimitive motion;
    motion.type = MotionType::DRUM;
    motion.robotic_drum_score = rds;
    return motion;
}

int BehaviorPlanner::find_motor_id(const std::string& motor_name) const {
    for (const auto &[id, motor] : robot.motors) {
        if (motor->name == motor_name) return id;
    }
    return -1;
}