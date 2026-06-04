#pragma once

#include <queue>

#include "common/motion_queue.hpp"

struct StateMotionPoint {
    double right_elbow;
    double left_elbow;

    double right_wrist;
    double left_wrist;
};

class StateMotionGenerator {
public:
    StateMotionGenerator();
    ~StateMotionGenerator();

    std::queue<StateMotionPoint> generate_motion(const std::vector<DrumEvent> rds, int num_point);
private:
};