#pragma once

#include <iostream>
#include <string>
#include <map>
#include <fstream>
#include <vector>
#include <algorithm>
#include <utility>

#include "common/motion_queue.hpp"  // MotionPrimitive
#include "common/control_queue.hpp"
#include "kinematics/kinematics_solver.hpp"

class TrajectoryGenerator {
public:
    TrajectoryGenerator(ControlQueue &controlQueueRef);
    ~TrajectoryGenerator();

    void generate_trajectory(const MotionPrimitive& motion);
 
private:
    ControlQueue &control_queue;

    KinematicsSolver solver;

    const double dt = 0.005;        // 데이터 시간 간격 5ms

    // TODO: last_q, last_qd, last_p_R, last_p_L 초기 위치로 초기화하기
    std::vector<double> last_q;     // 마지막 위치
    std::vector<double> last_qd;    // 마지막 속도

    std::vector<double> last_p_R;   // 마지막 위치
    std::vector<double> last_p_L;   // 마지막 위치

    ControlMode tmotor_control_mode = ControlMode::VEL;
    ControlMode wrist_control_mode = ControlMode::CST;
    ControlMode pedal_control_mode = ControlMode::CSP;
    
    void generate_joint_space_trajectory(const MotionPrimitive& motion);
    void generate_task_space_trajectory(const MotionPrimitive& motion);
    void generate_idle_trajectory();

    std::vector<ControlMode> get_modes();
    std::pair<std::vector<double>, std::vector<double>>
        sample(std::vector<double>& q0, std::vector<double>& q1, int n, int k, TrajectoryProfile profile);
    std::pair<std::vector<double>, std::vector<double>>
        sample_trapezoidal(std::vector<double>& q0, std::vector<double>& q1, int n, int k);
    std::pair<std::vector<double>, std::vector<double>>
        sample_cubic(std::vector<double>& q0, std::vector<double>& q1, int n, int k);
    std::pair<std::vector<double>, std::vector<double>>
        sample_quintic(std::vector<double>& q0, std::vector<double>& q1, int n, int k);
    std::pair<std::vector<double>, std::vector<double>>
        sample_cosine(std::vector<double>& q0, std::vector<double>& q1, int n, int k);
};