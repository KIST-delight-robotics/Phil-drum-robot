#include "trajectory/base_motion_generator.hpp"

BaseMotionGenerator::BaseMotionGenerator() {

}

BaseMotionGenerator::~BaseMotionGenerator() {

}

void BaseMotionGenerator::initialize() {
    using json = nlohmann::json;
 
    solver.initialize();

    const std::string config_path = "drumrobot/config/drum_coordinate.json";
    std::ifstream ifs(config_path);
    if (!ifs.is_open()) {
        std::cerr << "[BaseMotionGenerator] Failed to open config file: "
                  << config_path << "\n";
        return;
    }

    json root;
    try {
        ifs >> root;
    } catch (const json::parse_error& e) {
        std::cerr << "[BaseMotionGenerator] JSON parse error in "
                  << config_path << ": " << e.what() << "\n";
        return;
    }

    drum_coordinates.clear();

    for (const auto& inst : root.at("instruments")) {
        InstrumentCoordinate coord;

        int id = inst.at("id").get<int>();

        const auto& right = inst.at("right");
        const auto& left  = inst.at("left");

        auto right_pos = right.at("position");
        auto left_pos  = left.at("position");

        coord.right_position = {
            right_pos.at(0).get<double>(),
            right_pos.at(1).get<double>(),
            right_pos.at(2).get<double>()
        };
        coord.right_wrist_angle_deg = right.at("wrist_angle_deg").get<double>();

        coord.left_position = {
            left_pos.at(0).get<double>(),
            left_pos.at(1).get<double>(),
            left_pos.at(2).get<double>()
        };
        coord.left_wrist_angle_deg = left.at("wrist_angle_deg").get<double>();

        drum_coordinates[id] = coord;   // TODO: 저장은 (번호, 위치)로 하되 읽을 때, 악기 이름과 번호를 매칭할 수 있는 공유 자원 있으면 좋을 듯
    }

    std::cout << "[BaseMotionGenerator] Loaded " << drum_coordinates.size()
              << " drum coordinates from " << config_path << "\n";
}

std::queue<BaseMotionPoint> BaseMotionGenerator::generate_motion(std::vector<DrumEvent> rds, int num_point) {
    std::queue<BaseMotionPoint> out;
 
    if (rds.size() < 2 || num_point <= 0) {
        return out;
    }

    return out;
}