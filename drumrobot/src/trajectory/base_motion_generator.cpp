#include "trajectory/base_motion_generator.hpp"

BaseMotionGenerator::BaseMotionGenerator() {

}

BaseMotionGenerator::~BaseMotionGenerator() {

}

void BaseMotionGenerator::initialize() {
    using json = nlohmann::json;
 
    solver.initialize();

    
}

std::queue<BaseMotionPoint>
BaseMotionGenerator::generate_motion(std::vector<DrumEvent> rds, int num_point) {
    std::queue<BaseMotionPoint> out;
 
    if (rds.size() < 2 || num_point <= 0) {
        return out;
    }

    return out;
}