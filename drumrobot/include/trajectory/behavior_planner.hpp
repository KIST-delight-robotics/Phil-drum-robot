#pragma once

#include <vector>

#include "common/app_context.hpp"
#include "common/motion_queue.hpp"          // ParsedCommand
#include "trajectory/command_parser.hpp"    // MotionPrimitive

class BehaviorPlanner {
public:
    BehaviorPlanner(AppContext &ctxRef);
    ~BehaviorPlanner();

    std::vector<MotionPrimitive> generate_motion_sequence(const ParsedCommand& parsed);
private:

    AppContext &ctx;
};