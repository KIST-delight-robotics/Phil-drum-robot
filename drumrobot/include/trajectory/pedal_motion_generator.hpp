#pragma once

#include <queue>
#include <cmath>

#include "common/motion_queue.hpp"
#include "common/robot_config.hpp"

struct PedalMotionPoint {
    double right;
    double left;
};

class PedalMotionGenerator {
public:
    PedalMotionGenerator();
    ~PedalMotionGenerator();

    std::queue<PedalMotionPoint> generate_motion(const std::vector<DrumEvent> rds, int num_point);
private:
    enum class State {
        REST_TO_REST,
        REST_TO_HIT,
        HIT_TO_REST,
        HIT_TO_HIT
    };

    struct BassParam {
        double t_stay;
        double t_lift;
        double t_hit;
    };

    struct HihatParam {
        double t_settling;
        double t_lift;
        double t_hit;
        double t_splash;
    };

    PedalMotionGenerator::State get_state(bool cur, bool next);
    PedalMotionGenerator::BassParam get_bass_param(double t0, double t1);
    PedalMotionGenerator::HihatParam get_hihat_param(double t0, double t1);
    double get_bass_angle(double t, BassParam bp, State bs);
    double get_hihat_angle(double t, HihatParam hp, State hs, bool is_splash = false);
    double cosine_profile(double qi, double qf, double ti, double tf, double t);
};