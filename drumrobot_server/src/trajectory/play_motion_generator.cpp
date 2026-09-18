#include "trajectory/play_motion_generator.hpp"

#include <sstream>

PlayMotionGenerator::PlayMotionGenerator(AppContext &ctxRef)
    : ctx(ctxRef) {
}

PlayMotionGenerator::~PlayMotionGenerator() {

}

void PlayMotionGenerator::initialize() {
    using json = nlohmann::json;
 
    solver.initialize();

    // 악기 파일 하나: 원 중심·반지름·법선·손목각(좌우 공용)·스캔 후보(팔 공용). 후보가 없는 악기는 중심 1개로 연주
    const std::string config_path = "drumrobot_server/config/drum_coordinate.json";
    std::ifstream ifs(config_path);
    if (!ifs.is_open()) {
        std::cerr << "[PlayMotionGenerator] Failed to open config file: "
                  << config_path << "\n";
        return;
    }

    std::map<int, InstrumentCoordinate> loaded;
    std::string scan_timestamp;
    std::ostringstream summary;
    try {
        json root;
        ifs >> root;
        scan_timestamp = root.value("scan_timestamp", "-");

        for (const auto& inst : root.at("instruments")) {
            InstrumentCoordinate coord;
            const std::string name = inst.at("name");
            const int id = instrument_name_to_id.at(name);

            const auto& c = inst.at("center");
            coord.center = {c.at(0).get<double>(), c.at(1).get<double>(), c.at(2).get<double>()};
            coord.wrist_angle = inst.at("wrist_angle_deg").get<double>() * M_PI / 180.0;
            // radius / normal 은 스캔 기록용 — 코드에서 쓰지 않아 읽지 않는다

            std::vector<std::array<double, 3>> candidates;   // 팔 공용 스캔 후보
            for (const auto& p : inst.value("candidates", json::array())) {
                candidates.push_back({p.at(0).get<double>(), p.at(1).get<double>(), p.at(2).get<double>()});
            }
            summary << name << " " << candidates.size() << ", ";

            // 팔 공용 후보 → 오른손 +x / 왼손 -x. 후보가 없으면 중심 1개 (선택 로직이 즉시 반환)
            if (candidates.empty()) candidates.push_back(coord.center);
            for (auto p : candidates) {
                p[0] += ROBOT::CANDIDATE_HAND_X_OFFSET;
                coord.right_candidate_positions.push_back(p);
                p[0] -= 2.0 * ROBOT::CANDIDATE_HAND_X_OFFSET;
                coord.left_candidate_positions.push_back(p);
            }
            loaded[id] = coord;
        }
        // 궤적 생성기는 좌표 없는 악기를 검사하지 않으므로 팔 악기(1~9) 는 여기서 모두 있어야 한다
        for (const auto& [name, id] : instrument_name_to_id) {
            if (id != 0 && !loaded.count(id)) throw std::runtime_error("instrument '" + name + "' 없음");
        }
    } catch (const std::exception& e) {
        // 파일 전체를 먼저 읽고(all-or-nothing) 이상이 없을 때만 적용
        std::cerr << "[PlayMotionGenerator] " << config_path << " 형식 이상: " << e.what() << " — 좌표 미적용\n";
        return;
    }

    drum_coordinates = loaded;
    std::cout << "[PlayMotionGenerator] Loaded " << drum_coordinates.size() << " drum coordinates from " << config_path
              << " (scan " << scan_timestamp << "), 후보: " << summary.str() << "\n";

    base_motion_generator.initialize(drum_coordinates);
    head_motion_generator.initialize(drum_coordinates);
}

bool PlayMotionGenerator::reset(std::array<double, ROBOT::NUM_JOINT>& q, int note_r, int note_l) {
    BaseMotionPoint b = base_motion_generator.reset(note_r, note_l);
    HeadMotionPoint h = head_motion_generator.reset(note_r);
    PedalMotionPoint p = pedal_motion_generator.reset();
    StateMotionPoint s = state_motion_generator.reset();

    std::array<double, 3> pR = b.right_position;
    std::array<double, 3> pL = b.left_position;
    double theta0 = b.waist;
    double theta7 = b.right_wrist;
    double theta8 = b.left_wrist;
    KinematicsSolver::IKResult result = solver.solve_ik(pR, pL, theta0, theta7, theta8, true);

    if (!result.success) {
        std::cerr << "[PlayMotionGenerator] RESET: Failed to solve inverse kinematics\n";
        return false;
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

    return true;
}

std::queue<std::array<double, ROBOT::NUM_JOINT>> PlayMotionGenerator::generate_motion(const std::vector<DrumEvent>& rds) {    
    if (rds.size() < 2) {
        std::cerr << "[PlayMotionGenerator] generate_motion: 드럼 이벤트가 부족합니다 (size="
                  << rds.size() << ", 최소 2개 필요). 해당 구간 생성을 건너뜁니다.\n";
        std::queue<std::array<double, ROBOT::NUM_JOINT>> empty_queue;
        return empty_queue;
    }

    // std::cout << "===== rds =====\n";
    // for (int i = 0; i < (int)rds.size(); i++) {
    //     std::cout << "[" << i << "] t: " << rds[i].t
    //               << "  note_R: " << rds[i].note_num_R
    //               << "  note_L: " << rds[i].note_num_L
    //               << "  vel_R: " << rds[i].velocity_R
    //               << "  vel_L: " << rds[i].velocity_L << "\n";
    // }

    std::queue<std::array<double, ROBOT::NUM_JOINT>> q_queue;
    auto [n, dt] = get_num_point(rds[0].t, rds[1].t);

    std::queue<BaseMotionPoint> base_motion = base_motion_generator.generate_motion(rds, n, dt);
    std::queue<HeadMotionPoint> head_motion = head_motion_generator.generate_motion(rds, n);
    std::queue<PedalMotionPoint> pedal_motion = pedal_motion_generator.generate_motion(rds, n, dt);
    std::queue<StateMotionPoint> state_motion = state_motion_generator.generate_motion(rds, n, dt);

    if (base_motion_generator.get_error() || state_motion_generator.get_error()) {
        std::queue<std::array<double, ROBOT::NUM_JOINT>> empty_queue;
        return empty_queue;
    }

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
        KinematicsSolver::IKResult result = solver.solve_ik(pR, pL, theta0, theta7, theta8, true);

        if (!result.success) {
            std::cerr << "[PlayMotionGenerator] PLAY: Failed to solve inverse kinematics\n";
            std::queue<std::array<double, ROBOT::NUM_JOINT>> empty_queue;
            return empty_queue;
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

std::pair<int, double> PlayMotionGenerator::get_num_point(double t0, double t1) {
    double n;

    // 한 라인의 데이터 개수 (5ms 단위)
    n = (t1 - t0) / ROBOT::DT_SECOND / ctx.play_speed_scale.load();
    round_sum += (int)(n * 10000) % 10000;
    if (round_sum >= 10000)
    {
        round_sum -= 10000;
        n++;
    }
    n = floor(n);

    double dt = ROBOT::DT_SECOND * ctx.play_speed_scale.load();

    return std::make_pair((int)n, dt);
}