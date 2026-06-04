#include "trajectory/state_motion_generator.hpp"

StateMotionGenerator::StateMotionGenerator() {

}

StateMotionGenerator::~StateMotionGenerator() {

}

std::queue<StateMotionPoint> StateMotionGenerator::generate_motion(const std::vector<DrumEvent> rds, int num_point) {
    std::queue<StateMotionPoint> out;
 
    if (rds.size() < 2 || num_point <= 0) {
        return out;
    }

    for (int i = 0; i < num_point; i++) {

    }

    return out;
}