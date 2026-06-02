#pragma once

#include <iostream>
#include <string>
#include <map>
#include <vector>
#include <algorithm>
#include <utility>
#include <queue>

#include "common/control_queue.hpp"
#include "common/motion_queue.hpp"  // MotionPrimitive
#include "common/robot_config.hpp"
#include "kinematics/kinematics_solver.hpp"
#include "trajectory/play_motion_generator.hpp"
#include "util/logger.hpp"

class TrajectoryGenerator {
public:
    TrajectoryGenerator(ControlQueue &controlQueueRef);
    ~TrajectoryGenerator();

    void initialize(const std::vector<double>& init_pose);
    void generate_trajectory(const MotionPrimitive& motion);
 
private:
    ControlQueue &control_queue;

    KinematicsSolver solver;
    PlayMotionGenerator play_motion_generator;

    std::vector<double> last_q;     // 마지막 위치
    std::vector<double> last_qd;    // 마지막 속도

    std::vector<double> last_p_R;   // 마지막 위치
    std::vector<double> last_p_L;   // 마지막 위치

    ControlMode tmotor_control_mode = ControlMode::VEL;
    ControlMode wrist_control_mode = ControlMode::CSP;
    ControlMode pedal_control_mode = ControlMode::CSP;
    
    void generate_joint_space_trajectory(const MotionPrimitive& motion);
    void generate_task_space_trajectory(const MotionPrimitive& motion);
    void generate_play_trajectory(const MotionPrimitive& motion);
    void generate_idle_trajectory();

    std::vector<ControlMode> get_modes();
    std::pair<std::vector<double>, std::vector<double>> sample(
        const std::vector<double>& q0,
        const std::vector<double>& q1,
        int n,
        int k,
        TrajectoryProfile profile
    );
    std::pair<std::vector<double>, std::vector<double>> sample_trapezoidal(
        const std::vector<double>& q0,
        const std::vector<double>& q1,
        int n,
        int k
    );
    std::pair<std::vector<double>, std::vector<double>> sample_cubic(
        const std::vector<double>& q0,
        const std::vector<double>& q1,
        int n,
        int k
    );
    std::pair<std::vector<double>, std::vector<double>> sample_quintic(
        const std::vector<double>& q0,
        const std::vector<double>& q1,
        int n,
        int k
    );
    std::pair<std::vector<double>, std::vector<double>> sample_cosine(
        const std::vector<double>& q0,
        const std::vector<double>& q1,
        int n,
        int k
    );
    void update_last_q(const std::vector<double>& q);
    void update_last_q(const std::vector<double>& p, const std::vector<double>& q);

    // ===== log =====
    Logger trajectory_log;
};