#include "trajectory/pedal_motion_generator.hpp"

PedalMotionGenerator::PedalMotionGenerator() {

}

PedalMotionGenerator::~PedalMotionGenerator() {

}

std::queue<PedalMotionPoint> PedalMotionGenerator::generate_motion(const std::vector<DrumEvent> rds, int num_point) {
    std::queue<PedalMotionPoint> out;
 
    if (rds.size() < 2 || num_point <= 0) {
        return out;
    }

    for (int i = 0; i < num_point; i++) {

    }

    return out;
}