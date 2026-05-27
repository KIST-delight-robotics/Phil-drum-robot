#pragma once

#include <vector>

#include "common/motion_queue.hpp"          // ParsedCommand
#include "trajectory/command_parser.hpp"    // MotionPrimitive

class BehaviorPlanner {
public:
    BehaviorPlanner();
    ~BehaviorPlanner();

    std::vector<MotionPrimitive> generate_motion_sequence(const ParsedCommand& parsed);
private:
};