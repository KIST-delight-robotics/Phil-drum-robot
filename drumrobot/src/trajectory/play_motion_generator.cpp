#include "trajectory/play_motion_generator.hpp"

PlayMotionGenerator::PlayMotionGenerator()
    : log("play") {
    std::vector<std::string> header = {
        "XR", "YR", "ZR", "XL", "YL", "ZL",
        "theta 0", "theta 7", "theta 8",
        "elbow R", "elbow L", "wrist R", "wrist L",
        "bass pedal", "hihat control", "head yaw", "head pitch"
    };
    log.set_header(header);
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

        std::string name = inst.at("name");
        int id = instrument_name_to_id.at(name);

        const auto& right = inst.at("right");
        const auto& left  = inst.at("left");

        auto right_pos = right.at("position");
        auto left_pos  = left.at("position");

        coord.right_position = {
            right_pos.at(0).get<double>(),
            right_pos.at(1).get<double>(),
            right_pos.at(2).get<double>()
        };
        coord.right_wrist_angle = right.at("wrist_angle_deg").get<double>() * M_PI / 180.0;

        coord.left_position = {
            left_pos.at(0).get<double>(),
            left_pos.at(1).get<double>(),
            left_pos.at(2).get<double>()
        };
        coord.left_wrist_angle = left.at("wrist_angle_deg").get<double>() * M_PI / 180.0;

        drum_coordinates[id] = coord;
    }

    std::cout << "[PlayMotionGenerator] Loaded " << drum_coordinates.size()
              << " drum coordinates from " << config_path << "\n";

    base_motion_generator.initialize(drum_coordinates);
    head_motion_generator.initialize(drum_coordinates);
}

std::array<double, ROBOT::NUM_JOINT> PlayMotionGenerator::reset() {
    BaseMotionPoint b = base_motion_generator.reset();
    HeadMotionPoint h = head_motion_generator.reset();
    PedalMotionPoint p = pedal_motion_generator.reset();
    StateMotionPoint s = state_motion_generator.reset();

    std::array<double, 3> pR = b.right_position;
    std::array<double, 3> pL = b.left_position;
    double theta0 = b.waist;
    double theta7 = b.right_wrist;
    double theta8 = b.left_wrist;
    KinematicsSolver::IKResult result = solver.ik_solve(pR, pL, theta0, theta7, theta8, true);

    if (!result.success) {
        std::cerr << "[PlayMotionGenerator] RESET: Failed to solve inverse kinematics\n";
        // TODO: 에러 처리 필요
    }

    std::array<double, ROBOT::NUM_JOINT> q;

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

    return q;
}

std::queue<std::array<double, ROBOT::NUM_JOINT>> PlayMotionGenerator::generate_motion(const std::vector<DrumEvent>& rds) {    
    std::queue<std::array<double, ROBOT::NUM_JOINT>> q_queue;

    if (rds.size() < 2) {
        std::cerr << "[PlayMotionGenerator] TODO: 에러 메세지 작성해야 함\n";
        return q_queue;
    }

    // std::cout << "===== rds =====\n";
    // for (int i = 0; i < (int)rds.size(); i++) {
    //     std::cout << "[" << i << "] t: " << rds[i].t
    //               << "  note_R: " << rds[i].note_num_R
    //               << "  note_L: " << rds[i].note_num_L
    //               << "  vel_R: " << rds[i].velocity_R
    //               << "  vel_L: " << rds[i].velocity_L << "\n";
    // }

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
        KinematicsSolver::IKResult result = solver.ik_solve(pR, pL, theta0, theta7, theta8, true);

        if (!result.success) {
            // std::cerr << "[PlayMotionGenerator] PLAY: Failed to solve inverse kinematics\n";
            // return q_queue; // TODO: 진행중인 동작을 멈춰야 함. 이렇게 중간에 끊고 다음거 이어서 만들면 계단 입력 나옴
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

        std::vector<double> values = {
            pR[0], pR[1], pR[2],
            pL[0], pL[1], pL[2],
            theta0, theta7, theta8,
            s.right_elbow, s.left_elbow,
            s.right_wrist, s.left_wrist,
            p.right, p.left, h.yaw, h.pitch
        };
        log.record(values);
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