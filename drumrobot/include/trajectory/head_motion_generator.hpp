#pragma once

#include <queue>

#include "common/motion_queue.hpp"

struct HeadMotionPoint {
    double yaw;
    double pitch;
};

class HeadMotionGenerator {
public:
    HeadMotionGenerator();
    ~HeadMotionGenerator();

    std::queue<HeadMotionPoint> generate_motion(std::vector<DrumEvent> rds, int num_point);
private:
};