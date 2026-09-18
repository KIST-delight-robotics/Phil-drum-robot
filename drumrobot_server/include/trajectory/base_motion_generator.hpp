#pragma once

#include <queue>
#include <vector>
#include <array>
#include <string>
#include <map>
#include <fstream>
#include <iostream>
#include <cmath>
#include <utility>
#include <bitset>

#include "nlohmann/json.hpp"

#include "common/motion_queue.hpp"
#include "common/robot_config.hpp"
#include "kinematics/kinematics_solver.hpp"
#include "util/logger.hpp"

struct BaseMotionPoint {
    std::array<double, 3> right_position;
    std::array<double, 3> left_position;

    double waist;
    double right_wrist;
    double left_wrist;
};

class BaseMotionGenerator {
public:
    BaseMotionGenerator();
    ~BaseMotionGenerator();
 
    void initialize(const std::map<int, InstrumentCoordinate>& coordinates);

    BaseMotionPoint reset(int note_r = 1, int note_l = 1);  // 초기 위치 기본값: 스네어
    std::queue<BaseMotionPoint> generate_motion(const std::vector<DrumEvent>& rds, int num_point, double dt);
    bool get_error();
 
private:
    KinematicsSolver solver;

    std::map<int, InstrumentCoordinate> drum_coordinates;

    enum class Arm { RIGHT, LEFT };
    const double HIT_DETECTION_THRESHOLD = 1.2;

    enum class State {
        REST_TO_REST,    // 이전 없음 -> 다음 없음 (계속 대기)
        REST_TO_HIT,     // 이전 없음 -> 다음 있음 (대기 -> 타격 진입)
        HIT_TO_REST,     // 이전 있음 -> 다음 없음 (타격 -> 대기 복귀)
        HIT_TO_HIT       // 이전 있음 -> 다음 있음 (연속 타격)
    };
    struct MotionContext {
        double last_t;                                  // 마지막 상태 전환 시각 (비행 중이면 출발 시각)
        int    last_start_instrument;                   // 출발 악기 (휴식 중이면 서 있는 악기)
        int    last_end_instrument;                     // 도착 악기 (휴식 중이면 last_start_instrument 와 같음)
        State  state;
        std::array<double, 3> last_start_position, last_end_position;   // 출발점 / 도착점 (휴식 중이면 같음)

        MotionContext(int init_instrument = 1)  // 초기 위치 기본값: 스네어
            : last_t(0.0), last_start_instrument(init_instrument), last_end_instrument(init_instrument),
              state(State::REST_TO_REST), last_start_position{}, last_end_position{} {}
    };

    struct MotionSegment {
        double t0, t1;          // 궤적 생성 구간
        double start_time, end_time;                  // 전체 궤적 기준 출발/도착 시간
        std::array<double, 3> start_position, end_position;
        double start_wrist_angle, end_wrist_angle;
        int start_instrument = 0, end_instrument = 0; // 출발/도착 악기 (반대손 좌우 순서 제약·유지 규칙용)
        MotionContext next_context;                   // 이전 시간, 이전 악기, 상태
    };

    struct WaistSegment {
        double t0, t1;      // 시간
        double q0, q1;      // 위치
        double v0, v1;      // 속도
    };

    MotionContext right_context;
    MotionContext left_context;

    std::pair<MotionSegment, MotionSegment> get_motion_segments(const std::vector<DrumEvent>& rds, const MotionContext& context_R, const MotionContext& context_L, bool verbose = false);
    double get_wrist_angle(int instrument);

// ========================================
    static constexpr int NUM_WAIST_SAMPLES = 1801;                                             // -90 ~ 90deg, 0.1deg 간격
    using WaistFeasibility = std::bitset<NUM_WAIST_SAMPLES>;                                   // i번째 허리각에서 IK 해가 있는지
    std::array<std::map<std::array<double, 3>, WaistFeasibility>, 2> candidate_waist_feasibility;    // [팔][후보 위치] → 그 팔 단독의 허리 가능 집합 (허리 관절 한계 AND). initialize에서 모든 후보에 대해 계산

    static int physical_instrument_id(int instrument) { return instrument == 9 ? 5 : instrument; }

    const std::vector<std::array<double, 3>>* get_candidate_positions(Arm arm, int instrument);                          // 없으면 nullptr
    WaistFeasibility compute_feasible_waist_range(Arm arm, const std::array<double, 3>& position, double wrist_angle);
    WaistFeasibility get_feasible_waist_range(Arm arm, const std::array<double, 3>& position, double wrist_angle);

    // 반대손 세그먼트(끝점 확정)가 주어졌을 때 내 후보별 평가. 점수는 허리 폭 하나, 나머지는 필터/우선순위
    struct CandidateScore {
        std::vector<int>  waist_width;     // 후보별 양팔 허리 폭 (0.1deg 틱 수)
        std::vector<bool> hand_order_ok;   // 같은 물리 악기 위 반대손과 R.x > L.x 만족
        int keep_idx = -1;                 // 같은 악기 반복이면 현 후보 인덱스 (반대손 무관), 아니면 -1
    };

    CandidateScore compute_candidate_scores(Arm arm, const MotionSegment& seg, const MotionSegment& other_seg);

    // 새 비행을 시작하는 팔의 end_position 선정. 한 손이면 select_hit_position, 양손이면 (i, j) 쌍 탐색 (폭 = 두 손 중 작은 쪽)
    std::pair<std::array<double, 3>, std::array<double, 3>> select_end_positions(const MotionSegment& seg_R, const MotionSegment& seg_L, bool need_select_R, bool need_select_L, bool verbose);
    std::array<double, 3> select_hit_position(Arm arm, const MotionSegment& seg, const MotionSegment& other_seg, bool verbose);
    std::pair<std::array<double, 3>, std::array<double, 3>> get_end_positions(const MotionSegment& seg_R, const MotionSegment& seg_L,
                                                                                    bool need_select_R, bool need_select_L, bool verbose);
    std::array<double, 3> get_fallback_position(Arm arm, int instrument);
// ========================================

    double time_scaling(double ti, double tf, double t);
    std::array<double, 3> make_path(const std::array<double, 3>& pi, const std::array<double, 3>& pf, double s);

    double prev_waist_angle;
    double cur_waist_angle;
    double cur_q0_min, cur_q0_max;
    double prev_t;

    BaseMotionGenerator::WaistSegment get_waist_segment(const std::vector<DrumEvent>& rds);
    std::pair<double, std::array<double, 2>> get_waist_angle(const std::vector<DrumEvent>& rds, int idx);
    std::pair<double, std::array<double, 2>> compute_waist_range(std::array<double, 3> pR, std::array<double, 3> pL, double the7, double the8);
    std::array<double, 2> compute_slopes(const std::array<double, 4> &q, const std::array<double, 4> &t);
    double cubic_hermite(double ta, double qa, double va, double tb, double qb, double vb, double t);

    // 연주 종료하는 에러
    bool base_end_error = false;
};