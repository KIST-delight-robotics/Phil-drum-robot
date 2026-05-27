#include "trajectory/trajectory_generator.hpp"

TrajectoryGenerator::TrajectoryGenerator() {

}

TrajectoryGenerator::~TrajectoryGenerator() {

}

std::vector<ControlData> TrajectoryGenerator::generate_trajectory(const MotionPrimitive& motion) {
    // // !motion_queue.empty()
    // MotionRequest req = motion_queue.pop();

    // switch (req.type) {
    // case MotionType::TRAPEZOIDAL:
    //     generate_trapezoidal_profile(last_q, req.q1, req.t_total);
    //     break;
    // case MotionType::DRUM_HIT:
    //     // generate_drum_hit(last_q, last_qd, req.q1, req.t_total);
    //     break;
    // }
}

void TrajectoryGenerator::generate_trapezoidal_profile(std::vector<double> q0, std::vector<double> q1, double t_total) {
    double t_acc = t_total / 4.0;   // 가속 구간 = 전체 시간의 1/4

    int n = q0.size();  // robot.num_joint
    int num_frames = static_cast<int>(t_total / dt) + 1;

    // // 모터 타입별 ControlMode 미리 결정
    // std::vector<ControlMode> mode_template(n, ControlMode::None);
    // for (auto &[id, motor] : robot.motors) {
    //     if (id >= n) continue;
    //     if (std::dynamic_pointer_cast<TMotor>(motor)) {
    //         mode_template[id] = ControlMode::POS;
    //     } else if (std::dynamic_pointer_cast<MaxonMotor>(motor)) {
    //         mode_template[id] = ControlMode::CSP;
    //     }
    // }

    // for (int k = 0; k < num_frames; k++) {
    //     double t = k * dt;
    //     double s;

    //     if (t < t_acc) {
    //         // 가속 구간
    //         s = 0.5 * (t * t) / (t_acc * (t_total - t_acc));
    //     } else if (t < t_total - t_acc) {
    //         // 등속 구간
    //         s = (t - t_acc / 2.0) / (t_total - t_acc);
    //     } else {
    //         // 감속 구간
    //         double t_rem = t_total - t;
    //         s = 1.0 - 0.5 * (t_rem * t_rem) / (t_acc * (t_total - t_acc));
    //     }
    //     s = std::clamp(s, 0.0, 1.0);

    //     ControlData data(n);
    //     data.mode = mode_template;
    //     for (int i = 0; i < n; i++) {
    //         data.q[i]  = q0[i] + s * (q1[i] - q0[i]);
    //         data.qd[i] = (q1[i] - q0[i]) / (t_total - t_acc) * 
    //                       (t < t_acc ? t / t_acc :
    //                        t < t_total - t_acc ? 1.0 :
    //                        (t_total - t) / t_acc);
    //     }

    //     control_queue.push(data);
    // }

    last_q  = q1;
    last_qd = std::vector<double>(n, 0.0);
}
