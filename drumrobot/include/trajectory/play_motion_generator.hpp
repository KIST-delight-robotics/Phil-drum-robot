#pragma once

#include <queue>
#include <vector>
#include <array>

#include "common/motion_queue.hpp"
#include "common/robot_config.hpp"
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

    std::queue<std::array<double, ROBOT::NUM_JOINT>> generate_motion(std::vector<DrumEvent> rds);

private:
    KinematicsSolver solver;

    BaseMotionGenerator base_motion_generator;
    HeadMotionGenerator head_motion_generator;
    PedalMotionGenerator pedal_motion_generator;
    StateMotionGenerator state_motion_generator;

    int get_num_point(double t0, double t1);

    int round_sum = 0.0;    // 소수점 오차 보정
};