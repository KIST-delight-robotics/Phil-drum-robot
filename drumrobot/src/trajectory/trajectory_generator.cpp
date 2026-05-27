#include "trajectory/trajectory_generator.hpp"

TrajectoryGenerator::TrajectoryGenerator() {
    solver.initialize();
}

TrajectoryGenerator::~TrajectoryGenerator() {

}

std::vector<ControlSetPoint> TrajectoryGenerator::generate_trajectory(const MotionPrimitive& motion) {
    switch (motion.type) {
    case MotionType::TRANSLATE:
        if (motion.space == TrajectorySpace::JOINT) {
            std::vector<ControlSetPoint> trajectory = generate_joint_space_trajectory(motion);
            return trajectory;
        } else if (motion.space == TrajectorySpace::TASK) {
            std::vector<ControlSetPoint> trajectory = generate_task_space_trajectory(motion);
            return trajectory;
        }
        break;
    case MotionType::IDLE:
        break;
    }

    std::vector<ControlSetPoint> err = {ControlSetPoint(1)};    // TODO: 에러 메세지 출력
    return err;
}

std::vector<ControlSetPoint> TrajectoryGenerator::generate_joint_space_trajectory(const MotionPrimitive& motion) {
    std::vector<ControlSetPoint> trajectory;

    std::vector<ControlMode> modes = get_modes();
    int num_point = static_cast<int>(motion.t_total / dt);

    std::vector<double> q0 = last_q;
    std::vector<double> q1 = motion.q_target;

    for (int k = 0; k < num_point; k++) {
        ControlSetPoint set_point = sample(q0, q1, num_point, k, motion.profile);
        set_point.mode = modes;
        trajectory.push_back(set_point);
    }

    last_q  = q1;
    last_qd = std::vector<double>(q0.size(), 0.0);

    return trajectory;
}

std::vector<ControlSetPoint> TrajectoryGenerator::generate_task_space_trajectory(const MotionPrimitive& motion) {

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

ControlSetPoint TrajectoryGenerator::sample(std::vector<double>& q0, std::vector<double>& q1, double n, double k, TrajectoryProfile profile) {
    ControlSetPoint set_point;

    switch (profile)
    {
    case TrajectoryProfile::TRAPEZOIDAL:
        set_point = sample_trapezoidal(q0, q1, n, k);
        break;
    case TrajectoryProfile::CUBIC:
        set_point = sample_cubic(q0, q1, n, k);
        break;
    case TrajectoryProfile::QUINTIC:
        set_point = sample_quintic(q0, q1, n, k);
        break;
    case TrajectoryProfile::COSINE:
        set_point = sample_cosine(q0, q1, n, k);
        break;
    }

    return set_point;
}

ControlSetPoint TrajectoryGenerator::sample_trapezoidal(std::vector<double>& q0, std::vector<double>& q1, double n, double k) {

}

ControlSetPoint TrajectoryGenerator::sample_cubic(std::vector<double>& q0, std::vector<double>& q1, double n, double k) {

}

ControlSetPoint TrajectoryGenerator::sample_quintic(std::vector<double>& q0, std::vector<double>& q1, double n, double k) {

}

ControlSetPoint TrajectoryGenerator::sample_cosine(std::vector<double>& q0, std::vector<double>& q1, double n, double k) {

}
