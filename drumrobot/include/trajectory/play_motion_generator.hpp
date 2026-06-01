#pragma once

#include <queue>
#include <vector>

#include "common/motion_queue.hpp"
#include "kinematics/kinematics_solver.hpp"
#include "trajectory/base_motion_generator.hpp"
#include "trajectory/head_motion_generator.hpp"
#include "trajectory/pedal_motion_generator.hpp"
#include "trajectory/state_motion_generator.hpp"

class PlayMotionGenerator {
public:
    PlayMotionGenerator();
    ~PlayMotionGenerator();

    void initialize();

    std::queue<std::vector<double>> generate_motion();

private:
    KinematicsSolver solver;

    BaseMotionGenerator base_motion_generator;
    HeadMotionGenerator head_motion_generator;
    PedalMotionGenerator pedal_motion_generator;
    StateMotionGenerator state_motion_generator;

    const int NUM_JOINT = 13;

    int get_num_point();
};