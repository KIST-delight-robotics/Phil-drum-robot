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
    inline constexpr double OPEN_HIHAT_Z_OFFSET     = 0.03;   // open hihat = closed hihat 의 중심·후보 z + 0.03 [m] (스캔이 drum_coordinate.json 에 기록)
}

// config/drum_coordinate.json 의 악기 한 항목 중 코드가 쓰는 값. 파일의 radius/normal 은 스캔 기록용이라 읽지 않는다
struct InstrumentCoordinate {
    std::array<double, 3> center{};           // 드럼 원 중심 [m, 서버 좌표계] — 대표점(후보 없을 때)·머리 방향 기준
    double                wrist_angle = 0.0;  // 타격 순간 손목각 [rad], 좌우 공용

    // 팔별 타격점 = 파일의 팔 공용 후보 x ± CANDIDATE_HAND_X_OFFSET (로드 시 생성). 후보가 없으면 중심 1개 → 선택 없음
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