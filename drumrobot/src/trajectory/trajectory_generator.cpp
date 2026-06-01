#include "trajectory/trajectory_generator.hpp"

TrajectoryGenerator::TrajectoryGenerator(ControlQueue &controlQueueRef)
    : control_queue(controlQueueRef), trajectory_log("trajectory") {
    
    std::vector<std::string> header = {
        "joint 0", "joint 1", "joint 2", "joint 3", "joint 4",
        "joint 5", "joint 6", "joint 7", "joint 8",
        "joint 9", "joint 10", "joint 11", "joint 12"
    };
    trajectory_log.set_header(header);
}

TrajectoryGenerator::~TrajectoryGenerator() {

}

void TrajectoryGenerator::initialize(const std::vector<double>& init_pose) {
    solver.initialize();
    play_motion_generator.initialize();

    update_last_q(init_pose);
}

void TrajectoryGenerator::generate_trajectory(const MotionPrimitive& motion) {
    switch (motion.type) {
    case MotionType::TRANSLATE:
        if (motion.space == TrajectorySpace::JOINT) {
            generate_joint_space_trajectory(motion);
        } else if (motion.space == TrajectorySpace::TASK) {
            generate_task_space_trajectory(motion);
        } else {
            std::cerr << "[TrajectoryGenerator] Unknown motion trajectory space\n";
        }
        break;
    case MotionType::DRUM:
        generate_play_trajectory(motion);
        break;
    case MotionType::IDLE:
        generate_idle_trajectory();
        break;
    default:
        std::cerr << "[TrajectoryGenerator] Unknown motion type\n";
        break;
    }
}

void TrajectoryGenerator::generate_joint_space_trajectory(const MotionPrimitive& motion) {
    std::vector<ControlMode> modes = get_modes();
    int num_point = static_cast<int>(motion.t_total / DT);

    std::vector<double> q0 = last_q;
    std::vector<double> q1 = motion.q_target;

    for (int k = 1; k <= num_point; k++) {
        auto [q, qd] = sample(q0, q1, num_point, k, motion.profile);

        ControlSetPoint set_point(NUM_JOINT);
        set_point.q = q;
        set_point.qd = qd;
        set_point.mode = modes;
        control_queue.push(set_point);

        trajectory_log.record(set_point.q);
    }

    update_last_q(q1);
}

void TrajectoryGenerator::generate_task_space_trajectory(const MotionPrimitive& motion) {
    std::vector<ControlMode> modes = get_modes();
    int num_point = static_cast<int>(motion.t_total / DT);

    std::vector<double> q0 = last_q;
    std::vector<double> q1 = motion.q_target;

    std::vector<double> p0 = {
        last_p_R[0], last_p_R[1], last_p_R[2],
        last_p_L[0], last_p_L[1], last_p_L[2]
    };

    std::vector<double> p1 = {
        motion.p_target_R[0], motion.p_target_R[1], motion.p_target_R[2],
        motion.p_target_L[0], motion.p_target_L[1], motion.p_target_L[2]
    };

    // 속도 계산을 위한 이전 관절각
    std::vector<double> prev_q = q0;

    for (int k = 1; k <= num_point; k++) {
        auto [q, qd] = sample(q0, q1, num_point, k, motion.profile);
        auto [p, pd] = sample(p0, p1, num_point, k, motion.profile);

        std::array<double, 3> pR = {p[0], p[1], p[2]};
        std::array<double, 3> pL = {p[3], p[4], p[5]};
        double theta0 = q[0];
        double theta7 = q[7];
        double theta8 = q[8];
        KinematicsSolver::IKResult result = solver.ik_solve(pR, pL, theta0, theta7, theta8);

        if (!result.success) {
            std::cerr << "[TrajectoryGenerator] Failed to solve inverse kinematics\n";
            return;
        }

        // 13차원 set_point 구성: IK 결과(0~8) + 관절 보간값(9~12)
        ControlSetPoint set_point(NUM_JOINT);
        for (int i = 0; i < 9; i++) {
            set_point.q[i] = result.q[i];   // 관절 0~8 (팔)
            set_point.qd[i] = (result.q[i] - prev_q[i]) / DT;
        }
        for (int i = 9; i < NUM_JOINT; i++) {
            set_point.q[i] = q[i];          // 관절 9~12 (페달, 머리)
            set_point.qd[i] = qd[i]; 
        }
        set_point.mode = modes;
        control_queue.push(set_point);

        prev_q = result.q;

        trajectory_log.record(set_point.q);
    }

    update_last_q(p1, q1);
}

void TrajectoryGenerator::generate_play_trajectory(const MotionPrimitive& motion) {
    std::vector<ControlMode> modes = get_modes();

    std::queue<std::vector<double>> play_motion = play_motion_generator.generate_motion();

    // 속도 계산을 위한 이전 관절각
    std::vector<double> prev_q = last_q;

    while (!play_motion.empty()) {
        std::vector<double> q = play_motion.front();
        play_motion.pop();

        ControlSetPoint set_point(NUM_JOINT);
        for (int i = 0; i < NUM_JOINT; i++) {
            set_point.q[i] = q[i];
            set_point.qd[i] = (q[i] - prev_q[i]) / DT;
        }
        
        set_point.mode = modes;
        control_queue.push(set_point);

        prev_q = q;

        trajectory_log.record(set_point.q);
    }
}

void TrajectoryGenerator::generate_idle_trajectory() {
    std::vector<ControlMode> modes = get_modes();
    double t_total = 1.0;
    int num_point = static_cast<int>(t_total / DT);

    std::vector<double> q0 = last_q;
    std::vector<double> q1 = last_q;    // TODO: 미세한 움직임 구현

    for (int k = 1; k <= num_point; k++) {
        auto [q, qd] = sample_cosine(q0, q1, num_point, k);

        ControlSetPoint set_point(NUM_JOINT);
        set_point.q = q;
        set_point.qd = qd;
        set_point.mode = modes;
        control_queue.push(set_point);

        trajectory_log.record(set_point.q);
    }

    update_last_q(q1);
}

std::vector<ControlMode> TrajectoryGenerator::get_modes() {
    std::vector<ControlMode> modes = {
        tmotor_control_mode,
        tmotor_control_mode,
        tmotor_control_mode,
        tmotor_control_mode,
        tmotor_control_mode,
        tmotor_control_mode,
        tmotor_control_mode,
        wrist_control_mode,
        wrist_control_mode,
        pedal_control_mode,
        pedal_control_mode,
        ControlMode::None,
        ControlMode::None
    };

    return modes;
}

std::pair<std::vector<double>, std::vector<double>> TrajectoryGenerator::sample(
    const std::vector<double>& q0,
    const std::vector<double>& q1,
    int n,
    int k,
    TrajectoryProfile profile
) {
    std::pair<std::vector<double>, std::vector<double>> result;

    switch (profile) {
    case TrajectoryProfile::TRAPEZOIDAL:
        result = sample_trapezoidal(q0, q1, n, k);
        break;
    case TrajectoryProfile::CUBIC:
        result = sample_cubic(q0, q1, n, k);
        break;
    case TrajectoryProfile::QUINTIC:
        result = sample_quintic(q0, q1, n, k);
        break;
    case TrajectoryProfile::COSINE:
        result = sample_cosine(q0, q1, n, k);
        break;
    }

    return result;
}

std::pair<std::vector<double>, std::vector<double>> TrajectoryGenerator::sample_trapezoidal(
    const std::vector<double>& q0,
    const std::vector<double>& q1,
    int n,
    int k
) {
    // 사다리꼴 속도 프로파일: 가속(1/4) - 등속(1/2) - 감속(1/4)
    // 정규화 시간 s = k / n  (구간 [0, 1))
    // 정규화 속도 = 위치 미분 / t_total  (qd = ds/dt · (q1 - q0), t_total = n·dt)

    int dim = q0.size();
    std::vector<double> q(dim, 0.0);
    std::vector<double> qd(dim, 0.0);
 
    double s = static_cast<double>(k) / static_cast<double>(n);   // 정규화 시간 [0, 1)
    double t_total = n * DT;                                      // 실제 전체 시간 [s]
 
    double s_pos;   // 정규화 위치 [0, 1]
    double s_vel;   // 정규화 속도 ds/d(s_time)
 
    const double t_acc = 0.25;      // 가속 구간 비율
    const double v_max = 4.0 / 3.0; // 최대 정규화 속도 (등속 구간 속도)
 
    if (s < t_acc) {
        // 가속 구간
        s_pos = 0.5 * v_max * (s * s) / t_acc;
        s_vel = v_max * (s / t_acc);
    } else if (s < 1.0 - t_acc) {
        // 등속 구간
        s_pos = 0.5 * v_max * t_acc + v_max * (s - t_acc);
        s_vel = v_max;
    } else {
        // 감속 구간
        double tau = 1.0 - s;
        s_pos = 1.0 - 0.5 * v_max * (tau * tau) / t_acc;
        s_vel = v_max * (tau / t_acc);
    }
    s_pos = std::clamp(s_pos, 0.0, 1.0);
 
    for (int i = 0; i < dim; i++) {
        double dq = q1[i] - q0[i];
        q[i]  = q0[i] + s_pos * dq;
        qd[i] = (s_vel / t_total) * dq;
    }
 
    return {q, qd};
}

std::pair<std::vector<double>, std::vector<double>> TrajectoryGenerator::sample_cubic(
    const std::vector<double>& q0,
    const std::vector<double>& q1,
    int n,
    int k
) {
    // 3차 다항식 보간: 양 끝점에서 속도 0
    // 정규화 시간 s = k / n  (구간 [0, 1))
    // 정규화 속도 = 위치 미분 / t_total  (qd = ds/dt · (q1 - q0), t_total = n·dt)
 
    int dim = q0.size();
    std::vector<double> q(dim, 0.0);
    std::vector<double> qd(dim, 0.0);
 
    double s = static_cast<double>(k) / static_cast<double>(n);
    double t_total = n * DT;
 
    double s_pos = 3.0 * s * s - 2.0 * s * s * s;
    double s_vel = 6.0 * s * (1.0 - s);
 
    for (int i = 0; i < dim; i++) {
        double dq = q1[i] - q0[i];
        q[i]  = q0[i] + s_pos * dq;
        qd[i] = (s_vel / t_total) * dq;
    }
 
    return {q, qd};
}

std::pair<std::vector<double>, std::vector<double>> TrajectoryGenerator::sample_quintic(
    const std::vector<double>& q0,
    const std::vector<double>& q1,
    int n,
    int k
) {
    // 5차 다항식 보간: 양 끝점에서 속도와 가속도 모두 0
    // 정규화 시간 s = k / n  (구간 [0, 1))
    // 정규화 속도 = 위치 미분 / t_total  (qd = ds/dt · (q1 - q0), t_total = n·dt)

    int dim = q0.size();
    std::vector<double> q(dim, 0.0);
    std::vector<double> qd(dim, 0.0);
 
    double s = static_cast<double>(k) / static_cast<double>(n);
    double t_total = n * DT;
 
    double s2 = s * s;
    double s3 = s2 * s;
    double s4 = s3 * s;
    double s5 = s4 * s;
 
    double s_pos = 10.0 * s3 - 15.0 * s4 + 6.0 * s5;
    double s_vel = 30.0 * s2 - 60.0 * s3 + 30.0 * s4;
 
    for (int i = 0; i < dim; i++) {
        double dq = q1[i] - q0[i];
        q[i]  = q0[i] + s_pos * dq;
        qd[i] = (s_vel / t_total) * dq;
    }
 
    return {q, qd};
}

std::pair<std::vector<double>, std::vector<double>> TrajectoryGenerator::sample_cosine(
    const std::vector<double>& q0,
    const std::vector<double>& q1,
    int n,
    int k
) {
    // 사인(코사인) 보간: s(t) = (1 - cos(πt)) / 2
    // 양 끝점에서 속도 0, 매끄러운 가속/감속
    // 정규화 시간 s = k / n  (구간 [0, 1))
    // 정규화 속도 = 위치 미분 / t_total  (qd = ds/dt · (q1 - q0), t_total = n·dt)
 
    int dim = q0.size();
    std::vector<double> q(dim, 0.0);
    std::vector<double> qd(dim, 0.0);
 
    double s = static_cast<double>(k) / static_cast<double>(n);
    double t_total = n * DT;
 
    double s_pos = 0.5 * (1.0 - std::cos(M_PI * s));
    double s_vel = 0.5 * M_PI * std::sin(M_PI * s);
 
    for (int i = 0; i < dim; i++) {
        double dq = q1[i] - q0[i];
        q[i]  = q0[i] + s_pos * dq;
        qd[i] = (s_vel / t_total) * dq;
    }
 
    return {q, qd};
}

void TrajectoryGenerator::update_last_q(const std::vector<double>& q) {
    last_q  = q;
    last_qd = std::vector<double>(q.size(), 0.0);

    KinematicsSolver::FKResult result = solver.fk_solve(last_q);

    if (!result.success) {
        std::cerr << "[TrajectoryGenerator] Failed to solve forward kinematics\n";
        return;
    }
    
    last_p_R.assign(result.pR.begin(), result.pR.end());
    last_p_L.assign(result.pL.begin(), result.pL.end());
}

void TrajectoryGenerator::update_last_q(const std::vector<double>& p, const std::vector<double>& q) {
    std::array<double, 3> pR = {p[0], p[1], p[2]};
    std::array<double, 3> pL = {p[3], p[4], p[5]};
    double theta0 = q[0];
    double theta7 = q[7];
    double theta8 = q[8];
    KinematicsSolver::IKResult result = solver.ik_solve(pR, pL, theta0, theta7, theta8);

    if (!result.success) {
        std::cerr << "[TrajectoryGenerator] Failed to solve inverse kinematics\n";
        return;
    }

    // 13차원 last_q 구성: IK 결과(0~8) + 관절 보간값(9~12)
    for (int i = 0; i < 9; i++) {
        last_q[i] = result.q[i];
    }
    for (int i = 9; i < NUM_JOINT; i++) {
        last_q[i] = q[i];
    }
    last_qd = std::vector<double>(NUM_JOINT, 0.0);

    last_p_R.assign(pR.begin(), pR.end());
    last_p_L.assign(pL.begin(), pL.end());
}