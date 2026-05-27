#pragma once

#include <iostream>
#include <string>
#include <map>
#include <fstream>
#include <vector>
#include <algorithm>

#include "common/motion_queue.hpp"  // MotionPrimitive
#include "common/control_queue.hpp" // ControlSetPoint
#include "kinematics/kinematics_solver.hpp"

class TrajectoryGenerator {
public:
    TrajectoryGenerator();
    ~TrajectoryGenerator();

    std::vector<ControlSetPoint> generate_trajectory(const MotionPrimitive& motion);
 
private:
    KinematicsSolver solver;

    const double dt = 0.005;        // 데이터 시간 간격 5ms

    // TODO: last_q, last_qd 초기 위치로 초기화하기
    std::vector<double> last_q;     // 마지막 위치
    std::vector<double> last_qd;    // 마지막 속도

    ControlMode tmotor_control_mode = ControlMode::VEL;
    ControlMode wrist_control_mode = ControlMode::CST;
    ControlMode pedal_control_mode = ControlMode::CSP;
    
    std::vector<ControlSetPoint> generate_joint_space_trajectory(const MotionPrimitive& motion);
    std::vector<ControlSetPoint> generate_task_space_trajectory(const MotionPrimitive& motion);

    std::vector<ControlMode> get_modes();
    ControlSetPoint sample(std::vector<double>& q0, std::vector<double>& q1, double n, double k, TrajectoryProfile profile);
    ControlSetPoint sample_trapezoidal(std::vector<double>& q0, std::vector<double>& q1, double n, double k);
    ControlSetPoint sample_cubic(std::vector<double>& q0, std::vector<double>& q1, double n, double k);
    ControlSetPoint sample_quintic(std::vector<double>& q0, std::vector<double>& q1, double n, double k);
    ControlSetPoint sample_cosine(std::vector<double>& q0, std::vector<double>& q1, double n, double k);
};