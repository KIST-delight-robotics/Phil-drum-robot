#pragma once

#include <iostream>
#include <vector>
#include <string>
#include <map>
#include <memory>
#include <cmath>
#include <fstream>

#include "nlohmann/json.hpp"

#include "common/app_context.hpp"
#include "common/command_queue.hpp"         // ParsedCommand, Opcode
#include "common/motion_queue.hpp"          // MotionPrimitive
#include "hardware/robot.hpp"
#include "util/audio_player.hpp"

class BehaviorPlanner {
public:
    BehaviorPlanner(AppContext &ctxRef, Robot &robotRef, AudioPlayer &audioRef);
    ~BehaviorPlanner();

    std::vector<MotionPrimitive> generate_motion_sequence(const ParsedCommand& parsed);
    void init_poses_from_json();

    std::map<std::string, std::vector<double>> poses;

private:
    AppContext &ctx;
    Robot &robot;

    // 마지막 목표 관절각 (다음 모션의 시작점이자 부분 명령(MOVE, LOOK)의 기준)
    std::vector<double> last_q_target;

    // 기본 이동 시간 [s]
    const double DEFAULT_MOVE_TIME = 3.0;
    const double LOOK_MOVE_TIME    = 1.0;
    const double GESTURE_MOVE_TIME = 1.0;
    const double DEFAULT_HIT_TIME  = 1.0;

    // Opcode별 핸들러
    std::vector<MotionPrimitive> handle_start();
    void handle_ready();
    std::vector<MotionPrimitive> handle_look(const std::vector<std::string>& args);
    std::vector<MotionPrimitive> handle_gesture(const std::vector<std::string>& args);
    std::vector<MotionPrimitive> handle_move(const std::vector<std::string>& args);
    std::vector<MotionPrimitive> handle_pose(const std::vector<std::string>& args);
    std::vector<MotionPrimitive> handle_hit(const std::vector<std::string>& args);
    std::vector<MotionPrimitive> handle_play(const std::vector<std::string>& args);
    std::vector<MotionPrimitive> handle_quit();

    // 헬퍼
    MotionPrimitive make_translate(const std::vector<double>& q_target, double t_total, TrajectoryProfile profile = TrajectoryProfile::COSINE);
    void set_last_q_target(const std::vector<double>& q);
    MotionPrimitive make_drum_hit(double t, int note_num);
    std::string trim_whitespace(const std::string &str);
    bool make_drum_event(const std::vector<std::string>& items, double bpm, double last_t, DrumEvent& out);
    MotionPrimitive make_drum_play(std::vector<DrumEvent> rds);
    int find_motor_id(const std::string& motor_name) const;
    double deg_to_rad(double deg) const { return deg * M_PI / 180.0; }

    // ===== audio =====
    AudioPlayer &audio_player;

    // play_list.json 로부터 로드한 id -> (악보명, 음악명) 매핑
    struct PlayEntry {
        std::string score;
        std::string audio;
        int init_note_r, init_note_l;
    };
    std::map<std::string, PlayEntry> play_list;

    void init_play_list_from_json();
};
