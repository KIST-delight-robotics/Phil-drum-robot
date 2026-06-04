#pragma once

#include <queue>
#include <vector>
#include <array>
#include <string>
#include <map>
#include <fstream>
#include <iostream>
#include <cmath>

#include "nlohmann/json.hpp"

#include "common/motion_queue.hpp"
#include "common/robot_config.hpp"
#include "kinematics/kinematics_solver.hpp"

struct BaseMotionPoint {
    double right_x;
    double right_y;
    double right_z;

    double left_x;
    double left_y;
    double left_z;

    double waist;
    double right_wrist;
    double left_wrist;
};

class BaseMotionGenerator {
public:
    BaseMotionGenerator();
    ~BaseMotionGenerator();
 
    void initialize();
 
    std::queue<BaseMotionPoint> generate_motion(std::vector<DrumEvent> rds, int num_point);
 
private:
    KinematicsSolver solver;

    struct InstrumentCoordinate {
        // 드럼 위치
        // 드럼을 치는 순간 손목 각도
        std::array<double, 3> right_position;
        double                right_wrist_angle_deg;
        std::array<double, 3> left_position;
        double                left_wrist_angle_deg;
    };

    std::map<int, InstrumentCoordinate> drum_coordinates;
};