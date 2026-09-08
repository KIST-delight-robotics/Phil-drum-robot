#pragma once

#include <iostream>
#include <vector>
#include <string>
#include <map>
#include <memory>
#include <cmath>
#include <fstream>
#include <algorithm>

#include "nlohmann/json.hpp"

#include "common/app_context.hpp"
#include "common/command_queue.hpp"         // ParsedCommand, Opcode
#include "common/motion_queue.hpp"          // MotionPrimitive
#include "hardware/robot.hpp"
#include "trajectory/collision_avoider.hpp"
#include "trajectory/improv_selector.hpp"
#include "util/audio_player.hpp"

class BehaviorPlanner {
public:
    BehaviorPlanner(AppContext &ctxRef, Robot &robotRef, AudioPlayer &audioRef);
    ~BehaviorPlanner();

    std::vector<MotionPrimitive> generate_motion_sequence(const ParsedCommand& parsed);
    void init_poses_from_json();

    // improv: 파일이 끝났을 때 다음 악보의 window들. 세션 없으면 빈 벡터.
    std::vector<MotionPrimitive> make_improv_chunk();

    // 이어치기 전환 (PLAYING 전용). front_rds: motion_queue 맨 앞 window의 이벤트들.
    // parked_note_r/l: 절단 직전 양손 파킹 악기 (MotionPlanner가 pop 시점마다 추적한 값).
    // 성공 시 커밋 구간 + 새 악보 window들을 반환하고 슬롯/play_id/improv 세션까지 갱신 완료.
    // 실패 시 빈 벡터를 반환하고 아무 상태도 바꾸지 않는다.
    std::vector<MotionPrimitive> make_switch_windows(const std::vector<DrumEvent>& front_rds,
                                                     const std::vector<std::string>& switch_args,
                                                     int parked_note_r, int parked_note_l);

    // 연주 abort(PAUSE/STOP) 통지. 반드시 save_pause_point() 이후에 호출.
    void on_play_abort(bool pause_requested);

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

    // 속도 배율 제한
    const double MIN_SCALE = 0.5;
    const double MAX_SCALE = 2.0;

    // 이어치기 커밋 구간 [악보 시간 s]. 팔이 이미 비행 중인 타격을 모두 착지시키는 범위
    // (base_motion_generator HIT_DETECTION_THRESHOLD와 같은 값 유지)
    const double COMMIT_HORIZON = 1.2;

    // Opcode별 핸들러
    std::vector<MotionPrimitive> handle_start();
    void handle_ready();
    std::vector<MotionPrimitive> handle_look(const std::vector<std::string>& args);
    std::vector<MotionPrimitive> handle_gesture(const std::vector<std::string>& args);
    std::vector<MotionPrimitive> handle_move(const std::vector<std::string>& args);
    std::vector<MotionPrimitive> handle_pose(const std::vector<std::string>& args);
    std::vector<MotionPrimitive> handle_hit(const std::vector<std::string>& args);
    std::vector<MotionPrimitive> handle_play(const std::vector<std::string>& args);
    void handle_pause();
    std::vector<MotionPrimitive> handle_resume();
    void handle_play_ctrl(const std::vector<std::string>& args);
    std::vector<MotionPrimitive> handle_quit();

    // make_score_windows가 읽고 쓰는 체이닝 상태 (선곡용 ChainContext + 직전 이벤트)
    struct ChainState : ImprovSelector::ChainContext {
        DrumEvent last_event;       // 다음 파일 rds[0] 시드 (누적 t 포함)
    };

    // improv 세션 상태. 전부 planner 스레드에서만 접근하므로 뮤텍스 불필요.
    struct ImprovState : ChainState {
        bool active = false;        // 파일이 끝나면 다음 파일을 이어붙일지
        std::string genre;          // 장르 디렉터리명 ("" = 전체 장르)
        ImprovSelector selector;
    };
    ImprovState improv;

    // improv 세션 시작. genre ""=전체, bpm 0=파일 bpm.
    std::vector<MotionPrimitive> make_improv_sequence(const std::string& id, const std::string& genre, double bpm);
    // 악보 한 파일 -> PLAYING window들 (START/END 없음). 실패/이벤트 없음이면 빈 벡터.
    // skip_silence: 머리/꼬리 무음 제거 (일반 곡은 오디오 싱크 때문에 false).
    // delay_first_hit: 첫 타격을 최소 한 박 늦춤 (팔 이동 시간 부족 곡, 선곡기가 판정).
    // start_row: 이 행 번호까지 건너뜀 (이어치기 재개는 슬롯 row 전달, 0 = 미사용).
    // min_first_gap: 첫 타격까지 최소 대기 [악보 시간 s] (이어치기 gap, 0 = 미사용).
    std::vector<MotionPrimitive> make_score_windows(const std::string& score_path, int start_bar,
                                                    ChainState& chain, bool skip_silence, bool delay_first_hit,
                                                    int start_row = 0, double min_first_gap = 0.0);

    // 이어치기 헬퍼
    double read_score_bpm(const std::string& score_path);
    bool scan_first_notes(const std::string& score_path, int start_row, int& first_note_r, int& first_note_l);
    std::vector<MotionPrimitive> make_switch_to_song(const std::string& id, ChainState& chain, bool& resumed);
    std::vector<MotionPrimitive> make_switch_to_improv(const std::vector<std::string>& args,
                                                       ImprovState& session, std::string& improv_id);

    // 양손 충돌 예측/회피 (window 복사 직전 공유 rds를 in-place 수정)
    CollisionAvoider collision_avoider;
    // 악보명 -> 파일 경로. data/scores/ 바로 아래 -> 없으면 1단계 하위 디렉터리 검색. 실패 시 "".
    std::string resolve_score_path(const std::string& score_name);

    // 헬퍼
    std::vector<MotionPrimitive> make_play_sequence(const std::string& id, int start_bar);
    MotionPrimitive make_translate(const std::vector<double>& q_target, double t_total, TrajectoryProfile profile = TrajectoryProfile::COSINE);
    void set_last_q_target(const std::vector<double>& q);
    MotionPrimitive make_drum_hit(double t, int note_num);
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
