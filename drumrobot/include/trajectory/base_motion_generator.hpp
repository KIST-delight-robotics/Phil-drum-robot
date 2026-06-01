#pragma once

#include <queue>

#include "common/motion_queue.hpp"

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

    std::queue<BaseMotionPoint> generate_motion();
private:
};