#include "trajectory/motion_planner.hpp"

MotionPlanner::MotionPlanner(AppContext &ctxRef, CommandQueue &commandQueueRef, ControlQueue &controlQueueRef, MotionQueue &motionQueueRef, Robot &robotRef)
    : ctx(ctxRef), command_queue(commandQueueRef), control_queue(controlQueueRef), motion_queue(motionQueueRef), robot(robotRef),
    behavior_planner(ctxRef, robotRef), trajectory_generator(control_queue) {}

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
                if (!ctx.recv_active) ctx.recv_active = true;
                if (!ctx.send_active) ctx.send_active = true;
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
        double diff = std::abs(init_pose[id] - motor->initial_joint_angle);

        if (diff > 1e-4) {
            std::cerr << "[MotionPlanner] init pose mismatch! "
                      << "joint " << id << " (" << motor->name << "): "
                      << "robot_poses.json="  << init_pose[id] * 180.0 / M_PI << "deg  "
                      << "motors.json="       << motor->initial_joint_angle * 180.0 / M_PI << "deg\n";
        }
    }
}

void MotionPlanner::parse_command(const std::string& cmd) {
    if (ctx.shutdown_requested.load()) return;

    ParsedCommand parsed = command_parser.parse(cmd);

    std::vector<MotionPrimitive> motion_sequence = behavior_planner.generate_motion_sequence(parsed);

    int n = motion_sequence.size();
    for (int i = 0; i < n; i++) {
        motion_queue.push(motion_sequence[i]);
    }
}

void MotionPlanner::schedule_idle_motion() {
    if (ctx.shutdown_requested.load()) return;

    MotionPrimitive idle_motion;

    idle_motion.type = MotionType::IDLE;    // IDLE을 MotionType에서 없애고 TRANSLATE(목표 관절각으로 이동)의 반복으로 구현 가능
    motion_queue.push(idle_motion);
}
