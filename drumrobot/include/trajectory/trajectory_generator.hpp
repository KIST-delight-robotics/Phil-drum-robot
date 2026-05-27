#pragma once

#include <iostream>
#include <string>
#include <map>
#include <fstream>
#include <vector>

#include "common/motion_queue.hpp"  // MotionPrimitive
#include "common/control_queue.hpp" // ControlData

class TrajectoryGenerator {
public:
    TrajectoryGenerator();
    ~TrajectoryGenerator();

    std::vector<ControlData> generate_trajectory(const MotionPrimitive& motion);
 
private:

    const double dt = 0.005;        // 데이터 시간 간격 5ms

    std::vector<double> last_q;     // 마지막 위치
    std::vector<double> last_qd;    // 마지막 속도

    void generate_trapezoidal_profile(std::vector<double> q0, std::vector<double> q1, double t_total);
};