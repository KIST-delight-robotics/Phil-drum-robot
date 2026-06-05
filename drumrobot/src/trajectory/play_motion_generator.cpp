#include "trajectory/play_motion_generator.hpp"

PlayMotionGenerator::PlayMotionGenerator() {

}

PlayMotionGenerator::~PlayMotionGenerator() {

}

void PlayMotionGenerator::initialize() {
    using json = nlohmann::json;
 
    solver.initialize();

    const std::string config_path = "drumrobot/config/drum_coordinate.json";
    std::ifstream ifs(config_path);
    if (!ifs.is_open()) {
        std::cerr << "[PlayMotionGenerator] Failed to open config file: "
                  << config_path << "\n";
        return;
    }

    json root;
    try {
        ifs >> root;
    } catch (const json::parse_error& e) {
        std::cerr << "[PlayMotionGenerator] JSON parse error in "
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

    std::cout << "[PlayMotionGenerator] Loaded " << drum_coordinates.size()
              << " drum coordinates from " << config_path << "\n";

    base_motion_generator.initialize(drum_coordinates);
    head_motion_generator.initialize(drum_coordinates);
}

std::vector<double> PlayMotionGenerator::reset() {
    base_motion_generator.reset();
    head_motion_generator.reset();
    state_motion_generator.reset();

    // TODO: 초기 위치 (스네어) 자세 반환하기
}

std::queue<std::array<double, ROBOT::NUM_JOINT>> PlayMotionGenerator::generate_motion(const std::vector<DrumEvent>& rds) {
    // rds[0]: 시작 자세
    // rds[1]: 목표 자세
    
    std::queue<std::array<double, ROBOT::NUM_JOINT>> q_queue;

    int n = get_num_point(rds[0].t, rds[1].t);

    std::queue<BaseMotionPoint> base_motion = base_motion_generator.generate_motion(rds, n);
    std::queue<HeadMotionPoint> head_motion = head_motion_generator.generate_motion(rds, n);
    std::queue<PedalMotionPoint> pedal_motion = pedal_motion_generator.generate_motion(rds, n);
    std::queue<StateMotionPoint> state_motion = state_motion_generator.generate_motion(rds, n);

    for (int i = 0; i < n; i++) {
        std::array<double, ROBOT::NUM_JOINT> q;

        BaseMotionPoint b = base_motion.front();
        base_motion.pop();

        HeadMotionPoint h = head_motion.front();
        head_motion.pop();

        PedalMotionPoint p = pedal_motion.front();
        pedal_motion.pop();

        StateMotionPoint s = state_motion.front();
        state_motion.pop();

        std::array<double, 3> pR = b.right_position;
        std::array<double, 3> pL = b.left_position;
        double theta0 = b.waist;
        double theta7 = b.right_wrist;
        double theta8 = b.left_wrist;
        KinematicsSolver::IKResult result = solver.ik_solve(pR, pL, theta0, theta7, theta8);

        if (!result.success) {
            std::cerr << "[PlayMotionGenerator] Failed to solve inverse kinematics\n";
            return q_queue; // TODO: 진행중인 동작을 멈춰야 함. 이렇게 중간에 끊고 다음거 이어서 만들면 계단 입력 나옴
        }

        for (int i = 0; i < 9; i++) {
            q[i] = result.q[i];   // 관절 0~8 (팔)
        }

        q[4] += s.right_elbow;
        q[6] += s.left_elbow;

        q[7] += s.right_wrist;
        q[8] += s.left_wrist;

        q[9] = p.right;
        q[10] = p.left;

        q[11] = h.yaw - q[0];
        q[12] = h.pitch;

        q_queue.push(q);
    }

    return q_queue;
}

int PlayMotionGenerator::get_num_point(double t0, double t1) {
    double n;

    // 한 라인의 데이터 개수 (5ms 단위)
    n = (t1 - t0) / ROBOT::DT_SECOND;
    round_sum += (int)(n * 10000) % 10000;
    if (round_sum >= 10000)
    {
        round_sum -= 10000;
        n++;
    }
    n = floor(n);

    return (int)n;
}