#pragma once

#include <queue>
#include <vector>
#include <array>
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
};