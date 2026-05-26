#include "trajectory/motion_planner.hpp"

MotionPlanner::MotionPlanner(AppContext &ctxRef, CommandQueue &commandQueueRef, ControlQueue &controlQueueRef, Robot &robotRef)
    : ctx(ctxRef), command_queue(commandQueueRef), control_queue(controlQueueRef), robot(robotRef) {}

MotionPlanner::~MotionPlanner() {}

void MotionPlanner::run() {
    initialize();

    while (ctx.running.load()) {
        // 명령 파싱 및 motion_queue에 적재
        if (!command_queue.empty()) {
            std::string cmd = command_queue.pop();
            parse_command(cmd);
        }

        // send_active 이 후 motion_queue가 없으면 자동 생성
        if (ctx.send_active.load() && motion_queue.empty()) {
            parse_command("idle");
        }

        // control_queue 잔량이 임계값 이하면 다음 모션 생성
        if ((control_queue.size() < threshold) && (!motion_queue.empty())) {
            generate_motion();
            if (!ctx.recv_active) ctx.recv_active = true;
            if (!ctx.send_active) ctx.send_active = true;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    ctx.running = false;
    std::cerr << "[MotionPlanner] 스레드 종료\n";
}

void MotionPlanner::initialize() {
    solver.initialize();

    init_poses_from_json();

    // 초기 위치
    last_q  = poses["init"];
    last_qd = std::vector<double>(robot.num_joint, 0.0);
}

void MotionPlanner::init_poses_from_json() {
    using json = nlohmann::json;

    std::ifstream f("drumrobot/config/robot_config.json");
    if (!f.is_open()) {
        std::cerr << "[MotionPlanner] Failed to open config/robot_config.json\n";
        return;
    }
    json config = json::parse(f);

    for (auto &[name, angles] : config["poses"].items()) {
        for (auto &a : angles) {
            poses[name].push_back(a.get<double>() * M_PI / 180.0);
        }
    }

    // init 포즈 vs 모터 initial_joint_angle 비교
    const auto &init_pose = poses["init"];
    for (auto &[id, motor] : robot.motors) {
        double diff = std::abs(init_pose[id] - motor->initial_joint_angle);

        if (diff > 1e-4) {
            std::cerr << "[MotionPlanner] init pose mismatch! "
                      << "joint " << id << " (" << motor->name << "): "
                      << "robot_config.json=" << init_pose[id] * 180.0 / M_PI << "deg  "
                      << "motors.json="       << motor->initial_joint_angle * 180.0 / M_PI << "deg\n";
        }
    }
}

void MotionPlanner::parse_command(const std::string &cmd) {
    if (ctx.shutdown_requested.load()) return;

    // 명령 파싱 및 motion_queue에 적재
    if (ctx.send_active) {
        if (cmd == "home") {
            motion_queue.push({MotionType::TRAPEZOIDAL, poses["home"], 4.0});
        } else if (cmd == "ready") {
            motion_queue.push({MotionType::TRAPEZOIDAL, poses["ready"], 4.0});
        } else if (cmd == "stop") {
            motion_queue.push({MotionType::TRAPEZOIDAL, last_q, 4.0});
        } else if (cmd == "quit" || cmd == "q") {
            motion_queue.push({MotionType::TRAPEZOIDAL, poses["shutdown"], 4.0});
            ctx.shutdown_requested = true;
        } else if (cmd == "idle") {
            motion_queue.push({MotionType::TRAPEZOIDAL, last_q, 1.0});
        } else {
            std::cout << "[MotionPlanner] 알 수 없는 명령: " << cmd << "\n";
        }
    } else {
        // send_active 전에는 시작/종료 명령만 처리
        if (cmd == "start") {
            motion_queue.push({MotionType::TRAPEZOIDAL, poses["home"], 4.0});
        } else if (cmd == "quit" || cmd == "q") {
            ctx.shutdown_requested = true;
        } else {
            std::cout << "[MotionPlanner] 알 수 없는 명령: " << cmd << "\n";
        }
    }   
}

void MotionPlanner::generate_motion() {
    MotionRequest req = motion_queue.front();
    motion_queue.pop();

    switch (req.type) {
    case MotionType::TRAPEZOIDAL:
        generate_trapezoidal_profile(last_q, req.q1, req.t_total);
        break;
    case MotionType::DRUM_HIT:
        // generate_drum_hit(last_q, last_qd, req.q1, req.t_total);
        break;
    }
}

void MotionPlanner::generate_trapezoidal_profile(std::vector<double> q0, std::vector<double> q1, double t_total) {
    double t_acc = t_total / 4.0;   // 가속 구간 = 전체 시간의 1/4

    int n = q0.size();  // robot.num_joint
    int num_frames = static_cast<int>(t_total / dt) + 1;

    // 모터 타입별 ControlMode 미리 결정
    std::vector<ControlMode> mode_template(n, ControlMode::None);
    for (auto &[id, motor] : robot.motors) {
        if (id >= n) continue;
        if (std::dynamic_pointer_cast<TMotor>(motor)) {
            mode_template[id] = ControlMode::POS;
        } else if (std::dynamic_pointer_cast<MaxonMotor>(motor)) {
            mode_template[id] = ControlMode::CSP;
        }
    }

    for (int k = 0; k < num_frames; k++) {
        double t = k * dt;
        double s;

        if (t < t_acc) {
            // 가속 구간
            s = 0.5 * (t * t) / (t_acc * (t_total - t_acc));
        } else if (t < t_total - t_acc) {
            // 등속 구간
            s = (t - t_acc / 2.0) / (t_total - t_acc);
        } else {
            // 감속 구간
            double t_rem = t_total - t;
            s = 1.0 - 0.5 * (t_rem * t_rem) / (t_acc * (t_total - t_acc));
        }
        s = std::clamp(s, 0.0, 1.0);

        ControlData data(n);
        data.mode = mode_template;
        for (int i = 0; i < n; i++) {
            data.q[i]  = q0[i] + s * (q1[i] - q0[i]);
            data.qd[i] = (q1[i] - q0[i]) / (t_total - t_acc) * 
                          (t < t_acc ? t / t_acc :
                           t < t_total - t_acc ? 1.0 :
                           (t_total - t) / t_acc);
        }

        control_queue.push(data);
    }

    last_q  = q1;
    last_qd = std::vector<double>(n, 0.0);
}