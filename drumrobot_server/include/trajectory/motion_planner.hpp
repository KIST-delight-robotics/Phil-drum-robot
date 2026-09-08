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
#include "trajectory/trajectory_generator.hpp"
#include "util/audio_player.hpp"
#include "util/logger.hpp"

class MotionPlanner {
public:
    MotionPlanner(AppContext &ctxRef, CommandQueue &commandQueueRef, ControlQueue &controlQueueRef, MotionQueue &motionQueueRef, Robot &robotRef, AudioPlayer &audioRef);
    ~MotionPlanner();

    void run();

private:
    AppContext &ctx;
    CommandQueue &command_queue;
    ControlQueue &control_queue;
    MotionQueue &motion_queue;

    Robot &robot;

    BehaviorPlanner behavior_planner;
    TrajectoryGenerator trajectory_generator;

    const long unsigned int threshold = 20;     // 궤적 생성 임계값

    void initialize();

    void plan_motions(const ParsedCommand& cmd);
    void schedule_idle_motion();
    void abort_play_motion();
    void save_pause_point();
    void do_switch();
    void track_parked_notes(const MotionPrimitive& motion);

    bool motion_done = true;

    // 이어치기 절단용 양손 파킹 악기 (DRUM window pop 시점마다 갱신)
    int parked_note_r = 1;
    int parked_note_l = 1;

    // ===== log =====
    Logger motion_log;

    void record_command(const ParsedCommand& cmd);
    void record_motion(const MotionPrimitive& motion);
};