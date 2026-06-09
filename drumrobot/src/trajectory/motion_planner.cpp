#include "trajectory/motion_planner.hpp"

MotionPlanner::MotionPlanner(AppContext &ctxRef, CommandQueue &commandQueueRef, ControlQueue &controlQueueRef, MotionQueue &motionQueueRef, Robot &robotRef)
    : ctx(ctxRef), command_queue(commandQueueRef), control_queue(controlQueueRef), motion_queue(motionQueueRef), robot(robotRef),
    behavior_planner(ctxRef, robotRef), trajectory_generator(control_queue), motion_log("motion_command") {}

MotionPlanner::~MotionPlanner() {}

void MotionPlanner::run() {
    initialize();

    while (ctx.running.load()) {
        if (auto cmd = command_queue.try_pop()) {
            // 명령이 있으면 파싱해서 motion_queue에 적재
            parse_command(*cmd);
        } else if (ctx.send_active.load() && motion_queue.empty()) {
            // send_active 이 후 motion_queue가 없으면 대기 모션
            schedule_idle_motion();
        }

        // control_queue 잔량이 임계값 이하면 다음 모션 생성
        if (control_queue.size() < threshold) {
            if (auto motion = motion_queue.try_pop()) {
                trajectory_generator.generate_trajectory(*motion);
                if (!ctx.recv_active.load()) ctx.recv_active = true;
                if (!ctx.send_active.load()) ctx.send_active = true;

                record_motion(*motion);
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    ctx.running = false;
    std::cerr << "[MotionPlanner] 스레드 종료\n";
}

void MotionPlanner::initialize() {
    behavior_planner.init_poses_from_json();

    // init 포즈 vs 모터 initial_joint_angle 비교
    const auto &init_pose = behavior_planner.poses["init"];
    for (auto &[id, motor] : robot.motors) {
        auto dxl = std::dynamic_pointer_cast<DynamixelMotor>(motor);
        if (dxl) continue;  // 다이나믹셀은 초기 위치 비교 안함

        double diff = std::abs(init_pose[id] - motor->initial_joint_angle);

        if (diff > 1e-4) {
            std::cerr << "[MotionPlanner] init pose mismatch! "
                      << "joint " << id << " (" << motor->name << "): "
                      << "robot_poses.json="  << init_pose[id] * 180.0 / M_PI << "deg  "
                      << "motors.json="       << motor->initial_joint_angle * 180.0 / M_PI << "deg\n";
        }
    }

    trajectory_generator.initialize(behavior_planner.poses);
}

void MotionPlanner::parse_command(const std::string& cmd) {
    if (ctx.robot_state.load() == RobotState::ShuttingDown) return; // 종료 상태가 되면 추가 명령 안받음

    ParsedCommand parsed = command_parser.parse(cmd);

    std::vector<MotionPrimitive> motion_sequence = behavior_planner.generate_motion_sequence(parsed);

    int n = motion_sequence.size();
    for (int i = 0; i < n; i++) {
        motion_queue.push(motion_sequence[i]);
    }

    record_command(cmd);
}

void MotionPlanner::schedule_idle_motion() {
    if (ctx.robot_state.load() == RobotState::ShuttingDown) {
        return; // 종료 상태가 되면 추가 명령 안받음
    } else if (ctx.robot_state.load() == RobotState::Init) {
        // 키 제거하기 전 현재 위치 유지
        MotionPrimitive standby_motion;

        standby_motion.type = MotionType::STANDBY;
        motion_queue.push(standby_motion);

        record_motion(standby_motion);
    } else if (ctx.robot_state.load() == RobotState::Idle) {
        // 대기 동작
        MotionPrimitive idle_motion;

        idle_motion.type = MotionType::IDLE;    // IDLE을 MotionType에서 없애고 TRANSLATE(목표 관절각으로 이동)의 반복으로 구현 가능
        motion_queue.push(idle_motion);

        record_motion(idle_motion);
    } else if (ctx.robot_state.load() == RobotState::Playing) {
        // 연주 종료
        std::cerr << "[MotionPlanner] 연주를 마쳤습니다.\n";
        ctx.robot_state = RobotState::Idle;
        MotionPrimitive idle_motion;

        idle_motion.type = MotionType::IDLE;    // IDLE을 MotionType에서 없애고 TRANSLATE(목표 관절각으로 이동)의 반복으로 구현 가능
        motion_queue.push(idle_motion);

        record_motion(idle_motion);
    }
}

// ===== log =====
void MotionPlanner::record_command(const std::string& cmd) {
    std::vector<std::string> log = {"CMD", cmd};
    motion_log.record(log);
}

void MotionPlanner::record_motion(const MotionPrimitive& motion) {
    std::vector<std::string> log = {"MOTION"};

    if (motion.type == MotionType::IDLE) log.push_back("idle");
    else if (motion.type == MotionType::TRANSLATE) log.push_back("translate");
    else if (motion.type == MotionType::DRUM) log.push_back("drum");
    else if (motion.type == MotionType::STANDBY) log.push_back("standby");

    // TODO: 모션 로그 더 구체적으로 작성하기

    motion_log.record(log);
}
