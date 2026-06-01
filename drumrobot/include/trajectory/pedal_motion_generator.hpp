#pragma once

#include <queue>

#include "common/motion_queue.hpp"

struct PedalMotionPoint {
    double right;
    double left;
};

class PedalMotionGenerator {
public:
    PedalMotionGenerator();
    ~PedalMotionGenerator();

    std::queue<PedalMotionPoint> generate_motion();
private:
};