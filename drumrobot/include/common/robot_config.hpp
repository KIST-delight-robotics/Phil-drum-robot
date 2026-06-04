#pragma once

#include <array>

namespace ROBOT {
    inline constexpr int    NUM_JOINT      = 13;
    inline constexpr int    NUM_INSTRUMENT = 10;
    inline constexpr double DT_SECOND      = 0.005;
}

struct InstrumentCoordinate {
    // 드럼 위치
    // 드럼을 치는 순간 손목 각도
    std::array<double, 3> right_position;
    double                right_wrist_angle_deg;
    std::array<double, 3> left_position;
    double                left_wrist_angle_deg;
};