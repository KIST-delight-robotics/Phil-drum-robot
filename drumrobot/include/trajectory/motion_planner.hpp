#pragma once

#include <iostream>
#include <thread>
#include <pthread.h>
#include <string>
#include <map>
#include <vector>

#include "common/app_context.hpp"
#include "common/command_queue.hpp"
#include "common/control_queue.hpp"
#include "common/motion_queue.hpp"
#include "hardware/robot.hpp"
#include "trajectory/behavior_planner.hpp"
#include "trajectory/command_parser.hpp"
#include "trajectory/trajectory_generator.hpp"

class MotionPlanner {
public:
    MotionPlanner(AppContext &ctxRef, CommandQueue &commandQueueRef, ControlQueue &controlQueueRef, MotionQueue &motionQueueRef, Robot &robotRef);
    ~MotionPlanner();

    void run();

private:
    AppContext &ctx;
    CommandQueue &command_queue;
    ControlQueue &control_queue;
    MotionQueue &motion_queue;

    Robot &robot;

    BehaviorPlanner behavior_planner;
    CommandParser command_parser;
    TrajectoryGenerator trajectory_generator;

    const long unsigned int threshold = 20;     // 궤적 생성 임계값

    void initialize();

    void parse_command(const std::string& cmd);
    void schedule_idle_motion();
};