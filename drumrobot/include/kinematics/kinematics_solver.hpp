#pragma once

#include <fstream>
#include <iostream>
#include <cmath>
#include <vector>
#include <map>

#include "nlohmann/json.hpp"

class KinematicsSolver {
public:
    KinematicsSolver();
    ~KinematicsSolver();

    void initialize();

    struct IKResult {
        std::vector<double> q;  // [θ0~θ8], rad
        bool success = false;   // 성공 여부
    };

    IKResult solve(
        const std::array<double, 3>& pR,
        const std::array<double, 3>& pL,
        double theta0,
        double theta7,
        double theta8
    ) const;

    bool check_joint_limits(const std::vector<double>& q) const;    // q 벡터가 모든 관절 한계 내에 있는지 확인

private:
    struct JointLimit {
        double min_angle;   // rad
        double max_angle;   // rad
    };

    struct LinkLength {
        double waist;       // 어깨 간격
        double upper_arm;   // 상완
        double forearm;     // 하완
        double stick;       // 스틱
    };

    std::map<int, JointLimit> joint_limits;
    LinkLength link_length;

    double get_effective_length(double theta_wrist) const;  // 하완+스틱 합성 링크 길이
    double get_effective_theta(double theta_wrist) const;   // 하완+스틱 합성 링크 방향각
};