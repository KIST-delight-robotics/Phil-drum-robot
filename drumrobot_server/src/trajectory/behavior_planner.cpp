#include "trajectory/behavior_planner.hpp"

#include "util/score_row.hpp"

#include <filesystem>
#include <system_error>

// 관절 ID 상수 (motors.json 참조)
namespace JointID {
    constexpr int WAIST            = 0;
    constexpr int R_SHOULDER_1     = 1;
    constexpr int L_SHOULDER_1     = 2;
    constexpr int R_SHOULDER_2     = 3;
    constexpr int R_ELBOW          = 4;
    constexpr int L_SHOULDER_2     = 5;
    constexpr int L_ELBOW          = 6;
    constexpr int R_WRIST          = 7;
    constexpr int L_WRIST          = 8;
    constexpr int R_PEDAL          = 9;
    constexpr int L_PEDAL          = 10;
    constexpr int HEAD_YAW         = 11;
    constexpr int HEAD_PITCH       = 12;
}

BehaviorPlanner::BehaviorPlanner(AppContext &ctxRef, Robot &robotRef, AudioPlayer &audioRef)
    : ctx(ctxRef), robot(robotRef), audio_player(audioRef) {
    // 초기 자세를 last_q_target으로 설정 (모터의 initial_joint_angle 사용)
    last_q_target.resize(ROBOT::NUM_JOINT, 0.0);
    for (const auto &[id, motor] : robot.motors) {
        if (id < ROBOT::NUM_JOINT) {
            last_q_target[id] = motor->initial_joint_angle;
        }
    }

    init_play_list_from_json();
    collision_avoider.initialize();
}

BehaviorPlanner::~BehaviorPlanner() {

}

std::vector<MotionPrimitive> BehaviorPlanner::generate_motion_sequence(const ParsedCommand& parsed) {
    std::vector<MotionPrimitive> sequence;

    if (!parsed.valid) {
        std::cerr << "[BehaviorPlanner] Invalid command\n";
        return sequence;
    }

    Opcode opcode = parsed.opcode;

    // ===== send_active 전 =====
    // 시작/종료 명령만 처리
    if (!ctx.send_active.load()) {
        if (opcode == Opcode::START) {
            return handle_start();
        } else if (opcode == Opcode::QUIT) {
            ctx.robot_state = RobotState::SHUTTINGDOWN;
            return sequence;
        } else {
            std::cerr << "[BehaviorPlanner] 수행할 수 없는 명령 (send_active=false): opcode="
                      << static_cast<int>(opcode) << "\n";
            return sequence;
        }
    }

    // ===== send_active 후 =====
    switch (opcode) {
        case Opcode::READY: {
            handle_ready();
            return sequence;
        }
        case Opcode::LOOK:    return handle_look(parsed.args);
        case Opcode::GESTURE: return handle_gesture(parsed.args);
        case Opcode::MOVE:    return handle_move(parsed.args);
        case Opcode::POSE:    return handle_pose(parsed.args);
        case Opcode::HIT:     return handle_hit(parsed.args);
        case Opcode::PLAY:    return handle_play(parsed.args);
        case Opcode::PAUSE: {
            handle_pause();
            return sequence;
        }
        case Opcode::RESUME:  return handle_resume();
        case Opcode::PLAY_CTRL: {
            handle_play_ctrl(parsed.args);
            return sequence;
        }
        case Opcode::QUIT:    return handle_quit();
        case Opcode::START:
            std::cerr << "[BehaviorPlanner] 이미 시작된 상태\n";
            return sequence;
        default:
            std::cerr << "[BehaviorPlanner] Unknown opcode\n";
            return sequence;
    }
}

void BehaviorPlanner::init_poses_from_json() {
    using json = nlohmann::json;

    std::ifstream f("drumrobot_server/config/robot_poses.json");
    if (!f.is_open()) {
        std::cerr << "[BehaviorPlanner] Failed to open config/robot_poses.json\n";
        return;
    }
    json config = json::parse(f);

    for (auto &[name, angles] : config["poses"].items()) {
        for (auto &a : angles) {
            poses[name].push_back(a.get<double>() * M_PI / 180.0);
        }
    }
}

void BehaviorPlanner::init_play_list_from_json() {
    using json = nlohmann::json;

    std::ifstream f("drumrobot_server/config/play_list.json");
    if (!f.is_open()) {
        std::cerr << "[BehaviorPlanner] Failed to open config/play_list.json\n";
        return;
    }
    json config = json::parse(f);

    for (auto &[id, entry] : config["play_list"].items()) {
        PlayEntry e;
        e.score = entry.value("score", "");
        e.audio = entry.value("audio", "");
        e.init_note_r = entry.value("init_note_r", 1);
        e.init_note_l = entry.value("init_note_l", 1);

        play_list[id] = e;
    }
}

// =============================================================
// Opcode별 핸들러
// =============================================================

// START: home 포즈로 이동
std::vector<MotionPrimitive> BehaviorPlanner::handle_start() {
    std::vector<MotionPrimitive> sequence;

    auto it = poses.find("home");
    if (it == poses.end()) {
        std::cerr << "[BehaviorPlanner] 'home' pose not found in robot_poses.json\n";
        return sequence;
    }

    sequence.push_back(make_translate(it->second, DEFAULT_MOVE_TIME));
    set_last_q_target(it->second);

    std::cout   << "\n========================================\n"
                << " 모터 토크 ON\n"
                << " 1. 고정 키를 모두 제거하세요.\n"
                << " 2. 제거 후 'READY' 명령을 입력하세요.\n"
                << "========================================\n\n";
    ctx.robot_state = RobotState::INIT;

    return sequence;
}

// READY: idle state로 변경
void BehaviorPlanner::handle_ready() {
    if (ctx.robot_state.load() == RobotState::INIT) {
        ctx.robot_state = RobotState::IDLE;
    }
}

// LOOK pan tilt : 머리 yaw, pitch 제어
std::vector<MotionPrimitive> BehaviorPlanner::handle_look(const std::vector<std::string>& args) {
    std::vector<MotionPrimitive> sequence;
    if (ctx.robot_state.load() != RobotState::IDLE) {
        std::cerr << "[BehaviorPlanner] LOOK rejected: only allowed in IDLE\n";
        return sequence;
    }

    try {
        double pan_deg  = std::stod(args[0]);
        double tilt_deg = std::stod(args[1]);

        // 마지막 목표를 복사해서 head 관절만 갱신
        std::vector<double> q_target = last_q_target;
        q_target[JointID::HEAD_YAW]   = deg_to_rad(pan_deg);
        q_target[JointID::HEAD_PITCH] = deg_to_rad(tilt_deg);

        sequence.push_back(make_translate(q_target, LOOK_MOVE_TIME));
        set_last_q_target(q_target);
    } catch (const std::exception &e) {
        std::cerr << "[BehaviorPlanner] LOOK parsing error: " << e.what() << "\n";
    }

    return sequence;
}

// GESTURE type : 미리 정의된 제스처 시퀀스
std::vector<MotionPrimitive> BehaviorPlanner::handle_gesture(const std::vector<std::string>& args) {
    std::vector<MotionPrimitive> sequence;
    if (ctx.robot_state.load() != RobotState::IDLE) {
        std::cerr << "[BehaviorPlanner] GESTURE rejected: only allowed in IDLE\n";
        return sequence;
    }

    const std::string& type = args[0];

    if (type == "nod") {
        // 끄덕임: 아래 → 위 → 정면
        std::vector<double> q;
        q = last_q_target; q[JointID::HEAD_PITCH] = deg_to_rad(20.0);
        sequence.push_back(make_translate(q, GESTURE_MOVE_TIME));
        q[JointID::HEAD_PITCH] = deg_to_rad(-20.0);
        sequence.push_back(make_translate(q, GESTURE_MOVE_TIME));
        q[JointID::HEAD_PITCH] = 0.0;
        sequence.push_back(make_translate(q, GESTURE_MOVE_TIME));
        set_last_q_target(q);
    }
    else if (type == "shake") {
        // 도리도리: 좌 → 우 → 정면
        std::vector<double> q;
        q = last_q_target; q[JointID::HEAD_YAW] = deg_to_rad(30.0);
        sequence.push_back(make_translate(q, GESTURE_MOVE_TIME));
        q[JointID::HEAD_YAW] = deg_to_rad(-30.0);
        sequence.push_back(make_translate(q, GESTURE_MOVE_TIME));
        q[JointID::HEAD_YAW] = 0.0;
        sequence.push_back(make_translate(q, GESTURE_MOVE_TIME));
        set_last_q_target(q);
    }
    else if (type == "wave" || type == "hi") {
        // 인사: 오른팔 들기 + 손목 흔들기
        // 1) 오른팔 인사 자세
        std::vector<double> q = last_q_target;
        q[JointID::R_SHOULDER_1] = deg_to_rad(45.0);
        q[JointID::R_SHOULDER_2] = deg_to_rad(45.0);
        q[JointID::R_ELBOW]      = deg_to_rad(90.0);
        q[JointID::R_WRIST]      = 0.0;
        q[JointID::HEAD_YAW]     = deg_to_rad(-20.0);
        q[JointID::HEAD_PITCH]   = deg_to_rad(-5.0);
        sequence.push_back(make_translate(q, DEFAULT_MOVE_TIME));

        // 2) 손목 좌우 흔들기 3회
        for (int i = 0; i < 3; i++) {
            q[JointID::R_WRIST] = deg_to_rad(25.0);
            sequence.push_back(make_translate(q, 0.4));
            q[JointID::R_WRIST] = deg_to_rad(-25.0);
            sequence.push_back(make_translate(q, 0.4));
        }
        // 복귀
        q[JointID::R_WRIST] = 0.0;
        sequence.push_back(make_translate(q, 0.4));
        set_last_q_target(q);
    }
    else if (type == "hurray" || type == "happy") {
        // 환호: 양팔 들기
        std::vector<double> q = last_q_target;
        q[JointID::R_SHOULDER_1] = deg_to_rad(60.0);
        q[JointID::L_SHOULDER_1] = deg_to_rad(120.0);
        q[JointID::R_SHOULDER_2] = deg_to_rad(65.0);
        q[JointID::L_SHOULDER_2] = deg_to_rad(65.0);
        q[JointID::R_ELBOW]      = deg_to_rad(95.0);
        q[JointID::L_ELBOW]      = deg_to_rad(95.0);
        q[JointID::R_WRIST]      = 0.0;
        q[JointID::L_WRIST]      = 0.0;
        q[JointID::HEAD_PITCH]   = deg_to_rad(-15.0);
        sequence.push_back(make_translate(q, DEFAULT_MOVE_TIME));
        set_last_q_target(q);
    }
    else {
        std::cerr << "[BehaviorPlanner] Unknown gesture: " << type << "\n";
    }

    return sequence;
}

// MOVE: [motor_name, angle_deg] [move_time]
std::vector<MotionPrimitive> BehaviorPlanner::handle_move(const std::vector<std::string>& args) {
    std::vector<MotionPrimitive> sequence;
    if (ctx.robot_state.load() != RobotState::IDLE) {
        std::cerr << "[BehaviorPlanner] MOVE rejected: only allowed in IDLE\n";
        return sequence;
    }

    if (args.empty()) {
        std::cerr << "[BehaviorPlanner] MOVE rejected: no arguments\n";
        return sequence;
    }

    try {
        std::vector<double> q_target = last_q_target;
        double move_time = DEFAULT_MOVE_TIME;
        size_t i = 0;
        bool any_applied = false;

        // (motor_name, angle_deg) 쌍을 순회하며 적용
        while (i + 1 < args.size()) {
            const std::string& motor_name = args[i];
            int motor_id = find_motor_id(motor_name);
            if (motor_id < 0) {
                std::cerr << "[BehaviorPlanner] Unknown motor name: " << motor_name << "\n";
                return sequence;  // 하나라도 잘못되면 전체 취소
            }

            double angle_deg = std::stod(args[i + 1]);
            q_target[motor_id] = deg_to_rad(angle_deg);
            any_applied = true;
            i += 2;
        }

        // 마지막에 홀수로 남은 인자가 있으면 move_time으로 해석
        if (i < args.size()) {
            move_time = std::stod(args[i]);
        }

        if (!any_applied) {
            std::cerr << "[BehaviorPlanner] MOVE rejected: no valid motor/angle pairs\n";
            return sequence;
        }

        sequence.push_back(make_translate(q_target, move_time));
        set_last_q_target(q_target);
    } catch (const std::exception &e) {
        std::cerr << "[BehaviorPlanner] MOVE parsing error: " << e.what() << "\n";
    }

    return sequence;
}

// POSE pose_name : 사전 정의 포즈로 이동
std::vector<MotionPrimitive> BehaviorPlanner::handle_pose(const std::vector<std::string>& args) {
    std::vector<MotionPrimitive> sequence;
    if (ctx.robot_state.load() != RobotState::IDLE) {
        std::cerr << "[BehaviorPlanner] POSE rejected: only allowed in IDLE\n";
        return sequence;
    }

    const std::string& pose_name = args[0];

    auto it = poses.find(pose_name);
    if (it == poses.end()) {
        std::cerr << "[BehaviorPlanner] Unknown pose: " << pose_name << "\n";
        return sequence;
    }

    sequence.push_back(make_translate(it->second, DEFAULT_MOVE_TIME, TrajectoryProfile::TRAPEZOIDAL));
    set_last_q_target(it->second);

    // shutdown 포즈로 이동하는 경우 종료 플래그 세팅
    if (pose_name == "shutdown") {
        ctx.robot_state = RobotState::SHUTTINGDOWN;
    }

    return sequence;
}

// HIT target : 드럼 타격
std::vector<MotionPrimitive> BehaviorPlanner::handle_hit(const std::vector<std::string>& args) {
    std::vector<MotionPrimitive> sequence;
    if (ctx.robot_state.load() != RobotState::IDLE) {
        std::cerr << "[BehaviorPlanner] HIT rejected: only allowed in IDLE\n";
        return sequence;
    }

    const std::string& target = args[0];

    if (instrument_name_to_id.find(target) != instrument_name_to_id.end()) {
        MotionPrimitive start; start.type = MotionType::DRUM; start.flag = PlayFlag::START;
        sequence.push_back(start);

        int id = instrument_name_to_id.at(target);
        sequence.push_back(make_drum_hit(DEFAULT_HIT_TIME, id));

        MotionPrimitive end; end.type = MotionType::DRUM; end.flag = PlayFlag::END;
        sequence.push_back(end);

        // DRUM END가 ready 포즈로 복귀시키므로 종료 자세는 ready다.
        auto ready_it = poses.find("ready");
        if (ready_it != poses.end()) {
            set_last_q_target(ready_it->second);
        } else {
            std::cerr << "[BehaviorPlanner] HIT: 'ready' pose not found; last_q_target 미갱신\n";
        }
    } else {
        std::cerr << "[BehaviorPlanner] Unknown target instrument: " << target << "\n";
        return sequence;
    }

    return sequence;
}

std::vector<MotionPrimitive> BehaviorPlanner::handle_play(const std::vector<std::string>& args) {
    std::string verb = args[0];
    std::transform(verb.begin(), verb.end(), verb.begin(), ::tolower);

    if (verb != "improv") {
        return make_play_sequence(args[0], 0);      // play_list ID
    }

    // ===== 즉흥 연주 인자 해석 =====
    std::string genre;
    if (args.size() >= 2) {
        genre = args[1];
    }
    if (genre.find('/') != std::string::npos || genre.find("..") != std::string::npos) {
        std::cerr << "[BehaviorPlanner] 잘못된 장르 이름: " << genre << "\n";
        return std::vector<MotionPrimitive>();
    }

    double improv_bpm = 0.0;    // 0 = bpm 미지정 -> 각 파일의 bpm 사용
    if (args.size() >= 3 && !args[2].empty()) {
        size_t parsed_len = 0;
        try {
            improv_bpm = std::stod(args[2], &parsed_len);
        } catch (const std::exception& e) {
            parsed_len = 0;
        }
        if (parsed_len != args[2].size() || improv_bpm < 1.0 || improv_bpm > 250.0) {
            std::cerr << "[BehaviorPlanner] 즉흥 : bpm은 1~250 사이의 숫자여야 합니다:"
                      << args[2] << "\n";
            return std::vector<MotionPrimitive>();
        }
    }

    // 상태 표시용 id. GET_STATUS가 '|'로 필드를 나누므로 구분자는 ':'를 쓴다.
    std::string improv_id = "improv:" + (genre.empty() ? std::string("all") : genre);
    if (improv_bpm > 0.0) {
        improv_id += ":" + args[2];
    }
    return make_improv_sequence(improv_id, genre, improv_bpm);
}

// PAUSE : 연주 일시정지. 재개 지점을 저장하는 abort 경로로 보낸다.
void BehaviorPlanner::handle_pause() {
    if (ctx.robot_state.load() != RobotState::PLAYING) {
        std::cerr << "[BehaviorPlanner] PAUSE rejected: only allowed in PLAYING\n";
        return;
    }

    ctx.pause_requested = true;
    ctx.play_abort = true;
    std::cerr << "[BehaviorPlanner] 일시정지 요청 -> 재개 지점 저장 후 ready 복귀\n";
}

// RESUME : 저장된 재개 지점부터 다시 연주
std::vector<MotionPrimitive> BehaviorPlanner::handle_resume() {
    std::vector<MotionPrimitive> sequence;
    if (ctx.robot_state.load() != RobotState::IDLE) {
        std::cerr << "[BehaviorPlanner] RESUME rejected: only allowed in IDLE\n";
        return sequence;
    }

    PausePoint point;
    {   // play_mutex scope
        std::lock_guard<std::mutex> lock(ctx.play_mutex);
        if (!ctx.pause_point.valid) {
            std::cerr << "[BehaviorPlanner] RESUME rejected: 저장된 재개 지점이 없습니다\n";
            return sequence;
        }
        point = ctx.pause_point;
    }

    if (point.improv) {
        std::cerr << "[BehaviorPlanner] 즉흥 재개: id=" << point.play_id << "\n";
        return make_improv_sequence(point.play_id, point.genre, point.bpm);
    } else {
        std::cerr << "[BehaviorPlanner] 재개: id=" << point.play_id << ", bar=" << point.bar << "\n";
        return make_play_sequence(point.play_id, point.bar);
    }
}

// 재개 구간에서 해당 손의 첫 타격 악기 번호를 찾는다. 없으면 0.
static int find_first_note(const std::vector<DrumEvent>& rds, bool is_right) {
    for (size_t i = 1; i < rds.size(); i++) {
        int raw_note = is_right ? rds[i].note_num_R : rds[i].note_num_L;
        int note = open_hihat_note(raw_note, rds[i].is_closed_hihat);
        if (note != 0) {
            return note;
        }
    }
    return 0;
}

// START의 준비 위치(init_note)를 window들에서 찾은 손별 첫 타격 악기로 바꾼다.
static void set_start_notes(MotionPrimitive& start, const std::vector<MotionPrimitive>& windows) {
    int first_note_r = 0;
    int first_note_l = 0;
    for (size_t w = 0; w < windows.size(); w++) {
        if (first_note_r == 0) {
            first_note_r = find_first_note(windows[w].robotic_drum_score, true);
        }
        if (first_note_l == 0) {
            first_note_l = find_first_note(windows[w].robotic_drum_score, false);
        }
        if (first_note_r != 0 && first_note_l != 0) {
            break;
        }
    }
    if (first_note_r != 0) {
        start.init_note_r = first_note_r;
    }
    if (first_note_l != 0) {
        start.init_note_l = first_note_l;
    }
}

// 악보를 읽어 연주 모션 시퀀스를 만든다. start_bar > 0 이면 그 마디부터 시작(재개).
std::vector<MotionPrimitive> BehaviorPlanner::make_play_sequence(const std::string& id, int start_bar) {
    std::vector<MotionPrimitive> sequence;
    if (ctx.robot_state.load() != RobotState::IDLE) {
        std::cerr << "[BehaviorPlanner] PLAY rejected: only allowed in IDLE\n";
        return sequence;
    }

    auto it = play_list.find(id);
    if (it == play_list.end()) {
        std::cerr << "[BehaviorPlanner] PLAY: 알 수 없는 id: " << id << "\n";
        return sequence;
    }
    const std::string& score_name = it->second.score;
    const std::string& audio_name = it->second.audio;

    std::string score_path = resolve_score_path(score_name);
    if (score_path.empty()) {
        std::cerr << "[BehaviorPlanner] PLAY: 악보 파일을 찾을 수 없습니다: " << score_name << "\n";
        return sequence;
    }

    if (start_bar > 0) {
        audio_player.clear_track();     // 재개는 무음 (음악 중간부터 재생은 미지원)
    } else {
        audio_player.set_track(audio_name);
    }

    MotionPrimitive start; start.type = MotionType::DRUM; start.flag = PlayFlag::START;
    start.init_note_r = it->second.init_note_r;
    start.init_note_l = it->second.init_note_l;

    // 일반 곡은 새 체인으로 t=0부터 시작. 무음 행은 음원 싱크에 필요해서 건너뛰지 않는다.
    ChainState song_chain;
    collision_avoider.reset(it->second.init_note_r, it->second.init_note_l);
    std::vector<MotionPrimitive> windows = make_score_windows(score_path, start_bar, song_chain, false, false);

    if (start_bar > 0) {
        if (windows.empty()) {
            std::cerr << "[BehaviorPlanner] PLAY: 재개 마디(" << start_bar
                      << ")가 악보 범위 밖입니다. 연주 없이 ready로 복귀합니다\n";
        } else {
            // 재개 시 init_note는 곡 도입부 설정값이 아닌 재개 구간의 첫 타격 악기로 바꾼다.
            set_start_notes(start, windows);
            std::cerr << "[BehaviorPlanner] 재개 준비 위치: R=" << start.init_note_r
                      << ", L=" << start.init_note_l << "\n";
        }
    }

    sequence.push_back(start);
    for (size_t w = 0; w < windows.size(); w++) {
        sequence.push_back(windows[w]);
    }

    MotionPrimitive end; end.type = MotionType::DRUM; end.flag = PlayFlag::END;
    sequence.push_back(end);

    // DRUM END가 ready 포즈로 복귀시키므로 종료 자세는 ready다.
    auto ready_it = poses.find("ready");
    if (ready_it != poses.end()) {
        set_last_q_target(ready_it->second);
    } else {
        std::cerr << "[BehaviorPlanner] PLAY: 'ready' pose not found; last_q_target 미갱신\n";
    }

    {   // play_mutex scope
        std::lock_guard<std::mutex> lock(ctx.play_mutex);
        ctx.play_id = id;
        ctx.pause_point.clear();            // 새 연주 시작 -> 이전 재개 지점 폐기
    }

    ctx.robot_state = RobotState::PLAYING;
    ctx.play_speed_scale = 1.0;
    return sequence;
}

// ===== 즉흥 연주 (improv) =====
// START/END는 세션당 1번씩. 파일 사이는 window로 이어 붙이고, 선곡은 ImprovSelector가 한다.

// 악보명 -> 파일 경로. 루트 바로 아래 -> 장르 하위 1단계 순서로 검색. 실패 시 "".
std::string BehaviorPlanner::resolve_score_path(const std::string& score_name) {
    const std::string scores_root = "drumrobot_server/data/scores";
    std::string flat_path = scores_root + "/" + score_name + ".txt";

    std::error_code fs_error;
    if (std::filesystem::is_regular_file(flat_path, fs_error)) {
        return flat_path;
    }

    std::filesystem::directory_iterator dir_iter(scores_root, fs_error);
    if (fs_error) {
        return "";
    }
    for (const std::filesystem::directory_entry& entry : dir_iter) {
        std::error_code sub_error;
        if (!entry.is_directory(sub_error) || sub_error) {
            continue;
        }
        std::string sub_path = entry.path().string() + "/" + score_name + ".txt";
        if (std::filesystem::is_regular_file(sub_path, sub_error)) {
            return sub_path;
        }
    }
    return "";
}

// 악보 파일 하나를 PLAYING window들로 만든다.
// 시간은 chain.last_event에서 이어지고, chain.session_bpm이 있으면 파일 bpm 대신 쓴다.
// skip_silence면 앞뒤 무음은 버리고 중간 쉼표만 남긴다.
std::vector<MotionPrimitive> BehaviorPlanner::make_score_windows(const std::string& score_path, int start_bar,
                                                                 ChainState& chain, bool skip_silence, bool delay_first_hit,
                                                                 int start_row, double min_first_gap) {
    std::vector<MotionPrimitive> windows;

    std::ifstream input_file;
    input_file.open(score_path);
    if (!input_file.is_open()) {
        std::cerr << "[BehaviorPlanner] 악보 파일을 열 수 없습니다: " << score_path << "\n";
        return windows;
    }

    std::vector<DrumEvent> rds;
    rds.push_back(chain.last_event);    // rds[0]: 직전 이벤트 시드
    int start_idx = 0, end_idx = 0;

    double cur_bpm = (chain.session_bpm > 0.0) ? chain.session_bpm : 100.0;     // override 없으면 파일 bpm(기본 100)
    double last_t = chain.last_event.t;     // 누적 시간 연속

    std::vector<DrumEvent> pending;     // 무음 행 보류 버퍼 (꼬리 무음이면 폐기)
    bool hit_seen = false;              // 이 파일에서 타격 행을 만났는지
    int row_num = 0;                    // 파일 내 데이터 행 순번 (skip된 행도 번호는 소모)

    std::string row;
    while (getline(input_file, row)) {
        std::vector<std::string> items = split_score_row(row);

        if (items.empty() || items[0].empty()) {
            continue;   // 빈 줄 무시
        }

        if (items[0] == "bpm") {
            if (chain.session_bpm > 0.0) continue;      // override 모드: 파일 bpm 무시
            if (items.size() < 2) continue;
            try { cur_bpm = std::stod(items[1]); } catch (...) { }
            continue;
        }
        if (items[0] == "end") {
            break;      // 꼬리 flush는 아래 공통 경로에서
        }

        DrumEvent ev;
        if (!make_drum_event(items, cur_bpm, last_t, ev)) break;
        row_num++;
        ev.row = row_num;
        if (static_cast<int>(ev.bar) < start_bar) {
            continue;   // 재개 지점 이전 줄은 건너뜀 (last_t 미누적)
        }
        if (start_row > 0 && ev.row <= start_row) {
            continue;   // 이어치기 재개: 이미 연주된 행 건너뜀 (last_t 미누적)
        }

        bool is_hit = (ev.note_num_R != 0 || ev.note_num_L != 0 || ev.is_kick);
        if (skip_silence && !is_hit) {
            if (!hit_seen) {
                continue;   // 머리 무음: 통째로 건너뜀 (last_t 미누적)
            }
            pending.push_back(ev);  // 중간 쉼표인지 꼬리 무음인지 아직 모름 -> 보류
            last_t = ev.t;
            continue;
        }

        if (!hit_seen) {
            // 첫 타격까지 최소 대기 확보 (한 박 / 이어치기 gap 중 큰 값)
            double min_gap = min_first_gap;
            if (delay_first_hit) {
                double beat_gap = 0.6 * 100.0 / cur_bpm;
                if (beat_gap > min_gap) {
                    min_gap = beat_gap;
                }
            }
            if (min_gap > 0.0 && ev.t - rds[0].t < min_gap) {
                ev.t = rds[0].t + min_gap;
            }
        }
        hit_seen = true;

        // 손 위치 추적 (다음 파일 선곡용, 좌표 기준 5/9 변환)
        int coord_note_r = open_hihat_note(ev.note_num_R, ev.is_closed_hihat);
        int coord_note_l = open_hihat_note(ev.note_num_L, ev.is_closed_hihat);
        if (coord_note_r != 0) {
            chain.cur_note_r = coord_note_r;
        }
        if (coord_note_l != 0) {
            chain.cur_note_l = coord_note_l;
        }

        // 타격이 나왔으니 보류한 무음은 중간 쉼표다 -> 함께 넣는다.
        pending.push_back(ev);
        for (size_t p = 0; p < pending.size(); p++) {
            rds.push_back(pending[p]);

            end_idx++;
            last_t = rds[end_idx].t;

            // 한 마디 분량(100bpm 기준 2.4초)이 차면 window로 자른다.
            if ((rds[end_idx].t - rds[start_idx].t) * cur_bpm / 100.0 >= 2.4) {
                collision_avoider.process_window(rds, start_idx, end_idx, cur_bpm);     // 충돌 회피 (복사 전 rds 수정)
                windows.push_back(make_drum_play(std::vector<DrumEvent>(rds.begin() + start_idx, rds.begin() + end_idx + 1)));
                start_idx++;
            }
        }
        pending.clear();
    }
    input_file.close();

    // 남은 꼬리 window flush (end 줄/EOF/파싱 실패 공통)
    while (start_idx < end_idx) {
        collision_avoider.process_window(rds, start_idx, end_idx, cur_bpm);     // 충돌 회피 (복사 전 rds 수정)
        windows.push_back(make_drum_play(std::vector<DrumEvent>(rds.begin() + start_idx, rds.end())));
        start_idx++;
    }

    if (end_idx == 0) {
        return windows;     // 이벤트 없음 (빈 파일 / 재개 마디가 범위 밖)
    }

    chain.last_event = rds.back();      // 다음 파일 rds[0] 시드 갱신
    chain.last_bpm = cur_bpm;           // 선곡 bpm 연속성용
    return windows;
}

// 즉흥 세션 시작 (재개도 새 세션으로 진입)
std::vector<MotionPrimitive> BehaviorPlanner::make_improv_sequence(const std::string& id, const std::string& genre,
                                                                   double bpm) {
    const std::string scores_root = "drumrobot_server/data/scores";

    std::vector<MotionPrimitive> sequence;
    if (ctx.robot_state.load() != RobotState::IDLE) {
        std::cerr << "[BehaviorPlanner] PLAY rejected: only allowed in IDLE\n";
        return sequence;
    }

    // 세션 초기화 (last_event는 기본값 = t 0에서 시작)
    improv = ImprovState();
    improv.genre = genre;
    improv.session_bpm = bpm;
    collision_avoider.reset(improv.cur_note_r, improv.cur_note_l);

    if (!improv.selector.load(scores_root, genre)) {
        std::cerr << "[BehaviorPlanner] 즉흥: 연주할 악보 목록을 만들지 못했습니다: "
                  << (genre.empty() ? "(전체)" : genre) << "\n";
        return sequence;
    }

    // 빈 파일이면 다음 파일로 (한 바퀴까지). 첫 곡은 손 위치 필터 없이 순환 선곡.
    std::vector<MotionPrimitive> windows;
    int try_count = improv.selector.score_count();
    while (windows.empty() && try_count > 0) {
        std::string score_path = improv.selector.next_score();
        windows = make_score_windows(score_path, 0, improv, true, false);
        try_count--;
    }
    if (windows.empty()) {
        std::cerr << "[BehaviorPlanner] 즉흥: 연주 가능한 악보가 없습니다: "
                  << (genre.empty() ? "(전체)" : genre) << "\n";
        return sequence;
    }

    // START: 준비 위치는 첫 타격 악기 위 (없는 손은 기본값 1=스네어 유지)
    MotionPrimitive start;
    start.type = MotionType::DRUM;
    start.flag = PlayFlag::START;
    set_start_notes(start, windows);

    sequence.push_back(start);
    for (size_t w = 0; w < windows.size(); w++) {
        sequence.push_back(windows[w]);
    }

    audio_player.clear_track();     // 즉흥 연주는 음원 없이 진행

    // 드럼 연주는 ready 포즈에서 시작해 (정지 시) ready 포즈로 복귀한다.
    if (poses.count("ready") != 0) {
        set_last_q_target(poses["ready"]);
    } else {
        std::cerr << "[BehaviorPlanner] 즉흥: 'ready' pose not found; last_q_target 미갱신\n";
    }

    {   // play_mutex scope
        std::lock_guard<std::mutex> lock(ctx.play_mutex);
        ctx.play_id = id;                   // 상태 표시용 (예: "improv:funk:100")
        ctx.pause_point.clear();            // 새 연주 시작 -> 이전 재개 지점 폐기
    }

    ctx.robot_state = RobotState::PLAYING;
    ctx.play_speed_scale = 1.0;
    improv.active = true;

    std::cerr << "[BehaviorPlanner] 즉흥 연주 시작: genre=" << (genre.empty() ? "(전체)" : genre)
              << ", bpm=";
    if (bpm > 0.0) {
        std::cerr << bpm;
    } else {
        std::cerr << "(파일값)";
    }
    std::cerr << ", 악보 " << improv.selector.score_count() << "개, 첫 곡="
              << improv.selector.current_name() << "\n";
    return sequence;
}

// 파일이 끝났을 때 다음 악보의 window들 (연주 종료 훅에서 호출). 세션 없으면 빈 벡터.
std::vector<MotionPrimitive> BehaviorPlanner::make_improv_chunk() {
    std::vector<MotionPrimitive> windows;
    if (!improv.active) {
        return windows;
    }

    // 빈/깨진 파일은 건너뜀 (한 바퀴까지만 시도)
    int try_count = improv.selector.score_count();
    while (windows.empty() && try_count > 0) {
        bool delay_first_hit = false;
        std::string score_path = improv.selector.next_score(improv, delay_first_hit);
        if (score_path.empty()) {
            break;
        }
        windows = make_score_windows(score_path, 0, improv, true, delay_first_hit);
        if (windows.empty()) {
            std::cerr << "[BehaviorPlanner] 즉흥: 악보를 건너뜁니다: " << score_path << "\n";
        }
        try_count--;
    }

    if (windows.empty()) {
        std::cerr << "[BehaviorPlanner] 즉흥: 연주 가능한 악보가 없어 세션을 종료합니다\n";
        improv.active = false;
        return windows;
    }

    std::cerr << "[BehaviorPlanner] 즉흥 체이닝: " << improv.selector.current_name() << "\n";
    return windows;
}

// ===== 이어치기 전환 (무정지 switch) =====
// front_rds[0] = 마지막으로 친 이벤트(절단면). COMMIT_HORIZON 이내 이벤트는 팔이 이미
// 그 타격을 향해 비행 중일 수 있으므로 폐기하지 않고 그대로 연주한 뒤 새 악보를 잇는다.

std::vector<MotionPrimitive> BehaviorPlanner::make_switch_windows(const std::vector<DrumEvent>& front_rds,
                                                                  const std::vector<std::string>& switch_args,
                                                                  int parked_note_r, int parked_note_l) {
    std::vector<MotionPrimitive> windows;
    if (switch_args.empty() || front_rds.size() < 2) {
        std::cerr << "[BehaviorPlanner] 전환: 인자/절단면 정보가 없습니다\n";
        return windows;
    }

    std::string verb = switch_args[0];
    std::transform(verb.begin(), verb.end(), verb.begin(), ::tolower);
    bool to_improv = (verb == "improv");

    // 축약형: 곡 id도 improv도 아닌데 장르 디렉터리 이름이면 improv 진입으로 해석
    // (switch funk == switch improv funk). 곡 id가 항상 우선.
    std::vector<std::string> improv_args = switch_args;
    if (!to_improv && play_list.find(switch_args[0]) == play_list.end() &&
        verb.find('/') == std::string::npos && verb.find("..") == std::string::npos) {
        std::error_code dir_error;
        if (std::filesystem::is_directory("drumrobot_server/data/scores/" + verb, dir_error)) {
            to_improv = true;
            improv_args = {"improv", verb};
            std::cerr << "[BehaviorPlanner] 전환: 장르 축약형으로 해석 -> improv " << verb << "\n";
        }
    }

    // 커밋 구간: 절단면 + COMMIT_HORIZON 이내 이벤트 (front window의 이벤트는 이미 충돌 회피 처리된 버전)
    std::vector<DrumEvent> committed;
    int front_count = static_cast<int>(front_rds.size());
    for (int i = 0; i < front_count; i++) {
        if (front_rds[i].t - front_rds[0].t > COMMIT_HORIZON + 1e-6) {
            break;
        }
        committed.push_back(front_rds[i]);
    }
    int committed_count = static_cast<int>(committed.size());

    // 절단 시점 파킹 악기: pop 추적값을 커밋 구간 타격으로 갱신 (정보 없으면 스네어)
    int cur_note_r = parked_note_r;
    int cur_note_l = parked_note_l;
    for (int i = 0; i < committed_count; i++) {
        int coord_r = open_hihat_note(committed[i].note_num_R, committed[i].is_closed_hihat);
        int coord_l = open_hihat_note(committed[i].note_num_L, committed[i].is_closed_hihat);
        if (coord_r != 0) {
            cur_note_r = coord_r;
        }
        if (coord_l != 0) {
            cur_note_l = coord_l;
        }
    }
    if (cur_note_r < 1 || cur_note_r > 9) {
        cur_note_r = 1;
    }
    if (cur_note_l < 1 || cur_note_l > 9) {
        cur_note_l = 1;
    }

    // 나가는 곡 북마크는 커밋 마지막 이벤트 기준 (row/bar를 지우기 전에 확정)
    int bookmark_row = committed.back().row;
    int bookmark_bar = static_cast<int>(committed.back().bar);

    // 커밋 이벤트는 새 스트림에서 외부 이벤트다: row/bar를 지워 새 곡의 재개 지점으로
    // 오인되지 않게 한다 (커밋 구간 중 PAUSE 시 bar 0 = 처음부터 재개로 안전 강등).
    for (int i = 0; i < committed_count; i++) {
        committed[i].row = 0;
        committed[i].bar = 0;
    }

    // 절단 시점의 진행 bpm (선곡 연속성/한 박 하한 계산용)
    std::string outgoing_id;
    {   // play_mutex scope
        std::lock_guard<std::mutex> lock(ctx.play_mutex);
        outgoing_id = ctx.play_id;
    }
    bool outgoing_song = !improv.active;
    double cut_bpm = 100.0;
    if (improv.active) {
        cut_bpm = improv.last_bpm;
    } else {
        auto cur_it = play_list.find(outgoing_id);
        if (cur_it != play_list.end()) {
            std::string cur_path = resolve_score_path(cur_it->second.score);
            if (!cur_path.empty()) {
                cut_bpm = read_score_bpm(cur_path);
            }
        }
    }

    // 충돌 회피 상태를 절단 시점으로 시드 (실패 시 원복)
    std::pair<Eigen::VectorXd, Eigen::VectorXd> avoider_backup = collision_avoider.hand_state();
    collision_avoider.reset(cur_note_r, cur_note_l, committed.back().t);

    ChainState chain;
    chain.last_event = committed.back();
    chain.cur_note_r = cur_note_r;
    chain.cur_note_l = cur_note_l;
    chain.last_bpm = cut_bpm;
    chain.session_bpm = 0.0;

    // 대상 window 생성 (improv는 로컬 세션에 생성 후 성공 시에만 커밋)
    std::vector<MotionPrimitive> target_windows;
    ImprovState session;
    std::string new_id;
    bool resumed = false;

    if (to_improv) {
        session.last_event = chain.last_event;
        session.cur_note_r = chain.cur_note_r;
        session.cur_note_l = chain.cur_note_l;
        session.last_bpm = chain.last_bpm;
        target_windows = make_switch_to_improv(improv_args, session, new_id);
    } else {
        target_windows = make_switch_to_song(switch_args[0], chain, resumed);
        new_id = switch_args[0];
    }

    if (target_windows.empty()) {
        collision_avoider.set_hand_state(avoider_backup);
        return windows;     // 실패: 상태 무변, 기존 연주 계속
    }

    // 커밋 구간 window (한 이벤트씩 슬라이딩, 회피 재처리 없음)
    for (int i = 0; i + 1 < committed_count; i++) {
        windows.push_back(make_drum_play(std::vector<DrumEvent>(committed.begin() + i, committed.end())));
    }
    int target_count = static_cast<int>(target_windows.size());
    for (int i = 0; i < target_count; i++) {
        windows.push_back(target_windows[i]);
    }

    // 상태 커밋: 슬롯(§ 곡 북마크 하나 규칙) + play_id + improv 세션
    {   // play_mutex scope
        std::lock_guard<std::mutex> lock(ctx.play_mutex);
        if (to_improv) {
            if (outgoing_song && !outgoing_id.empty()) {
                ctx.pause_point.save(outgoing_id, bookmark_bar, bookmark_row);
            }
        } else {
            ctx.pause_point.clear();    // 곡 진입 = 슬롯 소비(재개) 또는 폐기(fresh)
        }
        ctx.play_id = new_id;
    }
    if (to_improv) {
        improv = std::move(session);
        improv.active = true;
        std::cerr << "[BehaviorPlanner] 전환 -> improv: 첫 곡=" << improv.selector.current_name()
                  << ", 파킹 R=" << cur_note_r << " L=" << cur_note_l << ", bpm=";
        if (improv.session_bpm > 0.0) {
            std::cerr << improv.session_bpm;
        } else {
            std::cerr << "(파일값)";
        }
        std::cerr << "\n";
    } else {
        improv = ImprovState();
        std::cerr << "[BehaviorPlanner] 전환 -> 곡 " << new_id
                  << (resumed ? " (이어치기 재개)" : " (처음부터)") << "\n";
    }
    if (outgoing_song && to_improv) {
        std::cerr << "[BehaviorPlanner] 곡 북마크 저장: " << outgoing_id
                  << ", row=" << bookmark_row << ", bar=" << bookmark_bar << "\n";
    }

    return windows;
}

// 전환 대상이 곡일 때: 슬롯 북마크가 일치하면 그 다음 행부터, 아니면 처음부터.
// 진입점이 고정이므로 gap(손 이동 시간)을 계산해 첫 타격을 늦춘다.
std::vector<MotionPrimitive> BehaviorPlanner::make_switch_to_song(const std::string& id, ChainState& chain, bool& resumed) {
    std::vector<MotionPrimitive> windows;
    resumed = false;

    auto it = play_list.find(id);
    if (it == play_list.end()) {
        std::cerr << "[BehaviorPlanner] 전환: 알 수 없는 id: " << id << "\n";
        return windows;
    }
    std::string score_path = resolve_score_path(it->second.score);
    if (score_path.empty()) {
        std::cerr << "[BehaviorPlanner] 전환: 악보 파일을 찾을 수 없습니다: " << it->second.score << "\n";
        return windows;
    }

    int start_row = 0;
    {   // play_mutex scope
        std::lock_guard<std::mutex> lock(ctx.play_mutex);
        if (ctx.pause_point.valid && !ctx.pause_point.improv &&
            ctx.pause_point.play_id == id && ctx.pause_point.row > 0) {
            start_row = ctx.pause_point.row;
        }
    }
    resumed = (start_row > 0);

    // gap [악보 시간 s]: 양손 이동 시간의 최대값 (speed scale로 환산). 한 박 하한은 delay_first_hit가 건다.
    double min_first_gap = 0.0;
    int first_note_r = 0;
    int first_note_l = 0;
    if (scan_first_notes(score_path, start_row, first_note_r, first_note_l) && improv.selector.ensure_coords()) {
        double speed_scale = ctx.play_speed_scale.load();
        double need_r = improv.selector.travel_sec(chain.cur_note_r, first_note_r, true) * speed_scale;
        double need_l = improv.selector.travel_sec(chain.cur_note_l, first_note_l, false) * speed_scale;
        min_first_gap = std::max(need_r, need_l);
    }

    windows = make_score_windows(score_path, 0, chain, true, true, start_row, min_first_gap);
    if (windows.empty()) {
        std::cerr << "[BehaviorPlanner] 전환: 연주할 구간이 없습니다 (row " << start_row
                  << " 이후): " << score_path << "\n";
    }
    return windows;
}

// 전환 대상이 improv일 때: 손 위치 매칭 선곡(chain-aware)으로 fresh 진입. 북마크 없음.
// session에는 호출 전에 절단 시점 chain 필드가 시드되어 있어야 한다.
std::vector<MotionPrimitive> BehaviorPlanner::make_switch_to_improv(const std::vector<std::string>& args,
                                                                    ImprovState& session, std::string& improv_id) {
    const std::string scores_root = "drumrobot_server/data/scores";
    std::vector<MotionPrimitive> windows;

    std::string genre;
    if (args.size() >= 2) {
        genre = args[1];
    }
    if (genre.find('/') != std::string::npos || genre.find("..") != std::string::npos) {
        std::cerr << "[BehaviorPlanner] 전환: 잘못된 장르 이름: " << genre << "\n";
        return windows;
    }

    // bpm 인자: 숫자 = 고정, 생략 = 직전 스트림 bpm 상속
    double improv_bpm = 0.0;
    if (args.size() >= 3 && !args[2].empty()) {
        size_t parsed_len = 0;
        try {
            improv_bpm = std::stod(args[2], &parsed_len);
        } catch (const std::exception& e) {
            parsed_len = 0;
        }
        if (parsed_len != args[2].size() || improv_bpm < 1.0 || improv_bpm > 250.0) {
            std::cerr << "[BehaviorPlanner] 전환: bpm은 1~250 사이의 숫자여야 합니다: " << args[2] << "\n";
            return windows;
        }
    }
    if (improv_bpm == 0.0 && session.last_bpm >= 1.0 && session.last_bpm <= 250.0) {
        improv_bpm = session.last_bpm;      // 템포 연속성: 직전 곡/improv의 bpm을 이어받음
    }

    session.genre = genre;
    session.session_bpm = improv_bpm;

    if (!session.selector.load(scores_root, genre)) {
        std::cerr << "[BehaviorPlanner] 전환: 연주할 악보 목록을 만들지 못했습니다: "
                  << (genre.empty() ? "(전체)" : genre) << "\n";
        return windows;
    }

    // fresh 진입도 체이닝 선곡: 절단 시점 손 위치에서 도달 가능한(가까운) 곡이 우선
    int try_count = session.selector.score_count();
    while (windows.empty() && try_count > 0) {
        bool delay_first_hit = false;
        std::string score_path = session.selector.next_score(session, delay_first_hit);
        if (score_path.empty()) {
            break;
        }
        windows = make_score_windows(score_path, 0, session, true, delay_first_hit);
        if (windows.empty()) {
            std::cerr << "[BehaviorPlanner] 전환: 악보를 건너뜁니다: " << score_path << "\n";
        }
        try_count--;
    }
    if (windows.empty()) {
        std::cerr << "[BehaviorPlanner] 전환: 연주 가능한 악보가 없습니다: "
                  << (genre.empty() ? "(전체)" : genre) << "\n";
        return windows;
    }

    improv_id = "improv:" + (genre.empty() ? std::string("all") : genre);
    if (improv_bpm > 0.0) {
        std::ostringstream bpm_text;
        bpm_text << improv_bpm;
        improv_id += ":" + bpm_text.str();
    }
    return windows;
}

// bpm 헤더 값. 헤더가 없거나 파싱 실패면 100 (make_score_windows 기본값과 동일).
double BehaviorPlanner::read_score_bpm(const std::string& score_path) {
    std::ifstream input_file(score_path);
    if (!input_file.is_open()) {
        return 100.0;
    }
    std::string row;
    while (std::getline(input_file, row)) {
        std::vector<std::string> items = split_score_row(row);
        if (items.empty() || items[0].empty()) {
            continue;
        }
        if (items[0] == "bpm") {
            if (items.size() >= 2) {
                try {
                    return std::stod(items[1]);
                } catch (...) {
                }
            }
            return 100.0;
        }
        break;      // 첫 데이터 행: bpm 헤더 없음
    }
    return 100.0;
}

// start_row 이후 양손의 첫 타격 악기 (5/9 변환 적용). 타격이 하나도 없으면 false.
bool BehaviorPlanner::scan_first_notes(const std::string& score_path, int start_row, int& first_note_r, int& first_note_l) {
    first_note_r = 0;
    first_note_l = 0;

    std::ifstream input_file(score_path);
    if (!input_file.is_open()) {
        return false;
    }

    int row_num = 0;
    std::string row;
    while (std::getline(input_file, row)) {
        std::vector<std::string> items = split_score_row(row);
        if (items.empty() || items[0].empty()) {
            continue;
        }
        if (items[0] == "bpm") {
            continue;
        }
        if (items[0] == "end" || items.size() < 8) {
            break;
        }

        int note_r = 0;
        int note_l = 0;
        int hihat = 0;
        try {
            note_r = std::stoi(items[2]);
            note_l = std::stoi(items[3]);
            hihat = std::stoi(items[7]);
        } catch (...) {
            break;
        }

        row_num++;
        if (start_row > 0 && row_num <= start_row) {
            continue;
        }

        if (first_note_r == 0 && note_r != 0) {
            first_note_r = open_hihat_note(note_r, hihat == 1);
        }
        if (first_note_l == 0 && note_l != 0) {
            first_note_l = open_hihat_note(note_l, hihat == 1);
        }
        if (first_note_r != 0 && first_note_l != 0) {
            break;
        }
    }
    return (first_note_r != 0 || first_note_l != 0);
}

// abort(PAUSE/STOP) 통지. save_pause_point() 이후에 불러야 한다.
void BehaviorPlanner::on_play_abort(bool pause_requested) {
    if (!improv.active) {
        return;
    }
    improv.active = false;

    if (!pause_requested) {
        std::cerr << "[BehaviorPlanner] 즉흥 세션 종료 (stop)\n";
        return;
    }

    // PAUSE: 재개 지점을 improv 정보(장르/bpm)로 다시 저장
    std::lock_guard<std::mutex> lock(ctx.play_mutex);
    ctx.pause_point.save(ctx.play_id, improv.genre, improv.session_bpm);
    std::cerr << "[BehaviorPlanner] 즉흥 재개 지점: genre=" << (improv.genre.empty() ? "(전체)" : improv.genre)
              << ", bpm=" << improv.session_bpm << "\n";
}

// PLAY_CTRL 드럼 연주 제어
void BehaviorPlanner::handle_play_ctrl(const std::vector<std::string>& args) {
    if (ctx.robot_state.load() != RobotState::PLAYING) {
        std::cerr << "[BehaviorPlanner] PLAY_CTRL rejected: only allowed in PLAYING\n";
        return;
    }

    const std::string& ctrl = args[0];

    if (ctrl == "stop") {
        ctx.pause_requested = false;    // stop은 재개 지점을 남기지 않는다
        ctx.play_abort = true;
        std::cerr << "[BehaviorPlanner] 연주 중지 요청 -> 잔여 모션 폐기 후 ready 복귀\n";
    }
    else if (ctrl == "speed") {
        if (args.size() < 2) {
            std::cerr << "[BehaviorPlanner] PLAY_CTRL speed: 배율 인자가 없습니다\n";
            return;
        }
        double scale;
        try {
            scale = std::stod(args[1]);
        } catch (const std::exception& e) {
            std::cerr << "[BehaviorPlanner] PLAY_CTRL speed: 잘못된 배율 값: " << args[1] << "\n";
            return;
        }
        double clamped = std::clamp(scale, MIN_SCALE, MAX_SCALE);
        ctx.play_speed_scale = clamped;
        std::cerr << "[BehaviorPlanner] 연주 속도 배율: " << clamped << "x";
        if (clamped != scale) {
            std::cerr << " (요청 " << scale << " 가 [" << MIN_SCALE << ", " << MAX_SCALE << "] 로 제한됨)";
        }
        std::cerr << "\n";
    }
    else if (ctrl == "switch") {
        if (args.size() < 2) {
            std::cerr << "[BehaviorPlanner] PLAY_CTRL switch: 대상 인자가 없습니다\n";
            return;
        }
        // 대상 검증/절단은 MotionPlanner가 motion_queue를 쥔 시점에 수행 (같은 스레드)
        ctx.switch_args.assign(args.begin() + 1, args.end());
        ctx.switch_pending = true;
        std::cerr << "[BehaviorPlanner] 이어치기 전환 요청: " << args[1] << "\n";
    }
    else {
        std::cerr << "[BehaviorPlanner] Unknown PLAY_CTRL: " << ctrl << "\n";
    }
}

std::vector<MotionPrimitive> BehaviorPlanner::handle_quit() {
    std::vector<MotionPrimitive> sequence;
    if (ctx.robot_state.load() != RobotState::IDLE) {
        std::cerr << "[BehaviorPlanner] QUIT rejected: only allowed in IDLE\n";
        return sequence;
    }

    // shutdown 포즈로 이동 후 종료 플래그 세팅
    auto it = poses.find("shutdown");
    if (it != poses.end()) {
        sequence.push_back(make_translate(it->second, DEFAULT_MOVE_TIME));
        set_last_q_target(it->second);
    }
    ctx.robot_state = RobotState::SHUTTINGDOWN;
    return sequence;
}

// =============================================================
// 헬퍼
// =============================================================

MotionPrimitive BehaviorPlanner::make_translate(const std::vector<double>& q_target, double t_total, TrajectoryProfile profile) {
    MotionPrimitive motion;
    motion.type     = MotionType::TRANSLATE;
    motion.space    = TrajectorySpace::JOINT;
    motion.profile  = profile;
    motion.q_target = q_target;
    motion.t_total  = t_total;
    return motion;
}

void BehaviorPlanner::set_last_q_target(const std::vector<double>& q) {
    last_q_target = q;
    std::lock_guard<std::mutex> lk(ctx.last_q_mutex);
    ctx.last_q_target_snapshot = q;
}

MotionPrimitive BehaviorPlanner::make_drum_hit(double t, int note_num) {
    MotionPrimitive motion;
    motion.type     = MotionType::DRUM;

    DrumEvent Dummy;
    motion.robotic_drum_score.push_back(Dummy);     // rds[0]

    DrumEvent event;
    event.bar = 1;
    event.t = t;
    if (note_num == 0) {
        event.is_kick = true;
    } else  if (note_num == 2 || note_num == 3 || note_num == 6 || note_num == 7) {
        event.note_num_R = note_num;
        event.velocity_R = 5;
    } else {
        event.note_num_L = note_num;
        event.velocity_L = 5;
        if (note_num == 5) event.is_closed_hihat = true;
    }

    motion.robotic_drum_score.push_back(event);     // rds[1]
    return motion;
}

bool BehaviorPlanner::make_drum_event(const std::vector<std::string>& items, double bpm, double last_t, DrumEvent& out) {
    if (items.size() < 8) {
        std::cerr << "[BehaviorPlanner] PLAY: 악보 열 개수 부족 (" << items.size() << ")\n";
        return false;
    }
    try {
        out.bar             = std::stoi(items[0]);
        out.beat            = std::stod(items[1]);
        out.note_num_R      = std::stoi(items[2]);
        out.note_num_L      = std::stoi(items[3]);
        out.velocity_R      = std::stoi(items[4]);
        out.velocity_L      = std::stoi(items[5]);
        out.is_kick         = (std::stoi(items[6]) == 1);
        out.is_closed_hihat = (std::stoi(items[7]) == 1);
    } catch (const std::exception& e) {
        std::cerr << "[BehaviorPlanner] PLAY: 악보 숫자 파싱 실패: " << e.what() << "\n";
        return false;
    }
    out.t = out.beat * 100.0 / bpm + last_t;
    return true;
}

MotionPrimitive BehaviorPlanner::make_drum_play(std::vector<DrumEvent> rds) {
    MotionPrimitive motion;
    motion.type = MotionType::DRUM;
    motion.robotic_drum_score = rds;
    return motion;
}

int BehaviorPlanner::find_motor_id(const std::string& motor_name) const {
    for (const auto &[id, name] : robot.joint_names) {
        if (name == motor_name) return id;
    }
    return -1;
}