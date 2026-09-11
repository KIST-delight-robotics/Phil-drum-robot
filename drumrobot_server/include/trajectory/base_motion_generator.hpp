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
        State  state = State::REST_TO_REST;
        double last_t = 0.0;                        // 마지막 상태 전환 시각
        int    last_instrument = 1;                 // 팔이 있는/출발한 악기 (초기 기본값: 스네어)
        std::array<double, 3> last_position{};      // 팔이 있는/출발한 점 (last_instrument 후보 중 하나)
        int    target_instrument = 0;               // 비행 중 잠긴 목표 악기 (비행 상태에서만 유효)
        std::array<double, 3> target_position{};    // 비행 중 잠긴 목표 점
        double target_time = 0.0;                   // 비행 중 잠긴 도착(타격) 시각
    };

    struct MotionSegment {
        double t0, t1;          // 궤적 생성 구간
        double start_time, end_time;                  // 전체 궤적 기준 출발/도착 시간
        std::array<double, 3> start_position, end_position;
        double start_wrist_angle, end_wrist_angle;
        int start_instrument = 0, end_instrument = 0; // 출발/도착 악기 (반대손 좌우 순서 제약용)
        MotionContext next_context;                   // 이전 시간, 이전 악기, 상태
    };

    struct WaistSegment {
        double t0, t1;      // 시간
        double q0, q1;      // 위치
        double v0, v1;      // 속도
    };

    MotionContext right_context;
    MotionContext left_context;

    // other_arm_segment: 반대손의 현재 계획 (후보 선택 시 내 타격 시각의 반대손 위치 계산용)
    // verbose: 실제 궤적 생성 호출에서만 true (허리 룩어헤드의 재호출은 false → 선택 로그 1회)
    BaseMotionGenerator::MotionSegment get_motion_segment(const std::vector<DrumEvent>& rds, Arm arm, const MotionContext& context,
                                                          const MotionSegment& other_arm_segment, bool verbose = false);
    void note_to_target(int note_num, Arm arm, std::array<double, 3>& out_position, double& out_wrist_angle_deg);

    // ===== 후보점 실시간 선택 =====
    static constexpr int NUM_WAIST_SAMPLES = 1801;                                             // -90 ~ 90deg, 0.1deg 간격
    static double waist_sample_angle(int i) { return -0.5 * M_PI + M_PI / 1800.0 * i; }        // i번째 샘플의 허리각 (compute_waist_range와 공용)
    using WaistFeasibility = std::bitset<NUM_WAIST_SAMPLES>;                                   // i번째 허리각에서 IK 해가 있는지
    static int physical_instrument_id(int instrument) { return instrument == 9 ? 5 : instrument; }   // open/closed hihat = 같은 심벌

    // [팔][후보 위치] → 그 팔 단독의 허리 가능 집합 (허리 관절 한계 AND). initialize에서 모든 후보에 대해 계산
    // ponytail: 키가 위치만이라 손목각이 다른 악기가 같은 좌표를 가지면 충돌 — 그런 악기가 생기면 (위치, 손목각) 키로 확장
    std::array<std::map<std::array<double, 3>, WaistFeasibility>, 2> candidate_waist_feasibility;

    WaistFeasibility sweep_arm_waist_feasibility(Arm arm, const std::array<double, 3>& position, double wrist_angle);   // 1801 × solve_arm_ik
    WaistFeasibility get_arm_waist_feasibility(Arm arm, const std::array<double, 3>& position, double wrist_angle);     // 캐시 히트 아니면 sweep
    double get_wrist_angle(Arm arm, int instrument);                                                                     // 악기별 손목각 [rad]
    const std::vector<std::array<double, 3>>* get_candidate_positions(Arm arm, int instrument);                          // 없으면 nullptr
    MotionSegment make_segment_from_context(Arm arm, const MotionContext& context);                                      // 반대손 컨텍스트 → 정지/비행 세그먼트 뷰
    std::array<double, 3> select_hit_position(Arm arm, int instrument, double hit_time,
                                              const MotionSegment& other_arm_segment,
                                              int start_instrument, const std::array<double, 3>& start_position,
                                              bool verbose);
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