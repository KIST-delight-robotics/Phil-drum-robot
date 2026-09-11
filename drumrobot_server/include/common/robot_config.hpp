#pragma once

#include <array>
#include <string>
#include <unordered_map>
#include <vector>

namespace ROBOT {
    inline constexpr int    NUM_JOINT      = 13;
    inline constexpr int    NUM_INSTRUMENT = 10;
    inline constexpr double DT_SECOND      = 0.005;

    // 스캔 후보점 → 팔별 타격점 변환 (실기 튠 노브)
    inline constexpr double CANDIDATE_HAND_X_OFFSET = 0.02;   // 오른손 +x / 왼손 −x [m]
    inline constexpr double OPEN_HIHAT_Z_OFFSET     = 0.03;   // open hihat 후보 = closed hihat 후보 z + 0.03 [m] (스캔이 drum_candidates.json에 기록)
}

struct InstrumentCoordinate {
    // 드럼 위치
    // 드럼을 치는 순간 손목 각도
    std::array<double, 3> right_position;
    double                right_wrist_angle;
    std::array<double, 3> left_position;
    double                left_wrist_angle;

    // 스캔 후보점 (팔별, x 오프셋 적용 완료). 비어 있으면 BaseMotionGenerator가 대표점 1개로 채운다
    std::vector<std::array<double, 3>> right_candidate_positions;
    std::vector<std::array<double, 3>> left_candidate_positions;
};

static const std::unordered_map<std::string, int> instrument_name_to_id = {
    {"bass",         0},
    {"snare",        1},
    {"floor",        2},
    {"mid",          3},
    {"top",          4},
    {"closed hihat", 5},
    {"ride",         6},
    {"right crash",  7},
    {"left crash",   8},
    {"open hihat",   9},
};