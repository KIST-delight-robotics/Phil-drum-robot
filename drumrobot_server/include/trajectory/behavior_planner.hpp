#pragma once

#include <iostream>
#include <vector>
#include <string>
#include <map>
#include <memory>
#include <cmath>
#include <fstream>
#include <algorithm>
#include <array>

#include "nlohmann/json.hpp"

#include "common/app_context.hpp"
#include "common/command_queue.hpp"         // ParsedCommand, Opcode
#include "common/motion_queue.hpp"          // MotionPrimitive
#include "hardware/robot.hpp"
#include "kinematics/kinematics_solver.hpp"
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

    // POINT 명령의 사전 IK 검증·허리각 스윕용
    KinematicsSolver solver;

    // 마지막 목표 관절각 (다음 모션의 시작점이자 부분 명령(MOVE, LOOK)의 기준)
    std::vector<double> last_q_target;

    // 기본 이동 시간 [s]
    const double DEFAULT_MOVE_TIME = 3.0;
    const double LOOK_MOVE_TIME    = 1.0;
    const double GESTURE_MOVE_TIME = 1.0;
    const double DEFAULT_HIT_TIME  = 1.0;

    // POINT 명령 파라미터 (스틱끝 좌표 이동)
    const double POINT_APPROACH_H        = 0.10;    // 목표점 상공 접근 높이 [m]
    const double POINT_ASCEND_H          = 0.05;    // 이동 전 수직 상승 높이 [m]
    const double POINT_ASCEND_TIME       = 1.0;     // 상승 시간 [s]
    const double POINT_TRAVEL_TIME       = 3.0;     // 본 이동 시간 [s]
    const double POINT_DESCEND_TIME      = 1.2;     // 수직 하강 시간 [s]
    const double POINT_DEFAULT_OFFSET_CM = 2.0;     // 기본 z 오프셋 [cm]
    const double POINT_MAX_OFFSET_CM     = 20.0;    // 최대 z 오프셋 [cm]
    const double POINT_WRIST_DEG         = 10.0;    // 활성팔 손목각 고정값 [deg]
    const double POINT_IDLE_RESTORE_Z    = 0.0;     // 회전 후 유휴팔 높이 복귀를 허용하는 최저 팁 z [m] (전 악기 표면보다 위)
    const double POINT_WAIST_MARGIN      = 20.0 * M_PI / 180.0;  // 허리각 밴드 경계 여유 [rad]

    // 속도 배율 제한
    const double MIN_SCALE = 0.5;
    const double MAX_SCALE = 2.0;

    // Opcode별 핸들러
    std::vector<MotionPrimitive> handle_start();
    void handle_ready();
    std::vector<MotionPrimitive> handle_look(const std::vector<std::string>& args);
    std::vector<MotionPrimitive> handle_gesture(const std::vector<std::string>& args);
    std::vector<MotionPrimitive> handle_move(const std::vector<std::string>& args);
    std::vector<MotionPrimitive> handle_pose(const std::vector<std::string>& args);
    std::vector<MotionPrimitive> handle_hit(const std::vector<std::string>& args);
    std::vector<MotionPrimitive> handle_point(const std::vector<std::string>& args);
    std::vector<MotionPrimitive> handle_play(const std::vector<std::string>& args);
    void handle_pause();
    std::vector<MotionPrimitive> handle_resume();
    void handle_play_ctrl(const std::vector<std::string>& args);
    std::vector<MotionPrimitive> handle_quit();

    // 헬퍼
    std::vector<MotionPrimitive> make_play_sequence(const std::string& id, int start_bar);
    MotionPrimitive make_translate(const std::vector<double>& q_target, double t_total, TrajectoryProfile profile = TrajectoryProfile::COSINE);
    MotionPrimitive make_task_translate(const std::array<double, 3>& pR, const std::array<double, 3>& pL,
                                        const std::vector<double>& q_target, double t_total,
                                        TrajectoryProfile profile = TrajectoryProfile::COSINE);
    bool select_point_waist(bool active_is_right, const std::array<double, 3>& p_above,
                            const std::array<double, 3>& p_final, bool skip_descent,
                            const std::array<double, 9>& q_base, double wrist_active, double& out_theta0);
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
