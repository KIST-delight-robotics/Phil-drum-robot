#pragma once

#include <queue>
#include <vector>
#include <array>
#include <string>
#include <map>
#include <fstream>
#include <iostream>
#include <cmath>

#include "nlohmann/json.hpp"

#include "common/motion_queue.hpp"
#include "common/robot_config.hpp"
#include "kinematics/kinematics_solver.hpp"

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

    BaseMotionPoint reset();
    std::queue<BaseMotionPoint> generate_motion(const std::vector<DrumEvent>& rds, int num_point);
 
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
        double last_t = 0.0;        // 이전 시간
        int last_instrument = 1;    // 이전 악기 (초기 위치는 스네어)
        State state = State::REST_TO_REST;
    };

    struct MotionSegment {
        double t0, t1;          // 궤적 생성 구간
        double start_time, end_time;                  // 전체 궤적 기준 출발/도착 시간
        std::array<double, 3> start_position, end_position;
        double start_wrist_angle, end_wrist_angle;
        MotionContext next_context;                   // 이전 시간, 이전 악기, 상태
    };

    MotionContext right_context;
    MotionContext left_context;

    BaseMotionGenerator::MotionSegment get_motion_segment(const std::vector<DrumEvent>& rds, Arm arm);
    void note_to_target(int note_num, Arm arm, std::array<double, 3>& out_position, double& out_wrist_angle_deg);
    double time_scaling(double ti, double tf, double t);
    std::array<double, 3> make_path(const std::array<double, 3>& pi, const std::array<double, 3>& pf, double s);
};