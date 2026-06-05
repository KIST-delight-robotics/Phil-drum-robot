#pragma once

#include <queue>
#include <map>
#include <cmath>

#include "common/motion_queue.hpp"
#include "common/robot_config.hpp"

struct HeadMotionPoint {
    double yaw;
    double pitch;
};

class HeadMotionGenerator {
public:
    HeadMotionGenerator();
    ~HeadMotionGenerator();

    void reset();

    void initialize(const std::map<int, InstrumentCoordinate>& coordinates);

    std::queue<HeadMotionPoint> generate_motion(const std::vector<DrumEvent> rds, int num_point);
private:
    std::map<int, InstrumentCoordinate> drum_coordinates;

    double get_nod_ntensity(const std::vector<DrumEvent> rds);
    float get_nod_angle(double beat_of_line, double nod_intensity, int i, int n);

    int next_note = 0;
    int cur_note = 0;

    double prev_nod_intensity = 0.0;
    double beat_sum = 0.0;
};