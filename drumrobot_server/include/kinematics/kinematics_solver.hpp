#pragma once

#include <fstream>
#include <iostream>
#include <cmath>
#include <vector>
#include <map>
#include <utility>
#include <random>
#include <iomanip>

#include "nlohmann/json.hpp"

class KinematicsSolver {
public:
    KinematicsSolver();
    ~KinematicsSolver();

    void initialize();

    struct IKResult {
        std::array<double, 9> q;  // [θ0~θ8], rad
        bool success = false;   // 성공 여부
    };

    IKResult solve_ik(
        const std::array<double, 3>& pR,
        const std::array<double, 3>& pL,
        double theta0,
        double theta7,
        double theta8,
        bool print_err
    ) const;

    bool check_joint_limits(const std::array<double, 9>& q, bool print_err) const;    // q 벡터가 모든 관절 한계 내에 있는지 확인
    bool check_joint_limit(int joint, double value, bool print_err = false) const;    // 관절 하나의 한계 검사 (한계 미정의 관절은 통과)

    // ===== 팔 단위 IK (허리 각도가 주어졌을 때 한 팔만 해석) =====
    // solve_ik 는 두 팔의 solve_arm_ik 와 허리(θ0) 한계 검사의 합성이다:
    //   solve_ik.success == solve_arm_ik(R).success && solve_arm_ik(L).success && limit(θ0)
    // 따라서 팔별 "가능한 허리각 집합"을 따로 구해 AND 하면 양팔 동시 IK 가능 집합과 같다.
    enum class ArmSide { RIGHT, LEFT };

    struct ArmIKResult {
        double shoulder1 = 0.0;   // θ1(R) / θ2(L): 수평 회전
        double shoulder2 = 0.0;   // θ3(R) / θ5(L): 수직 평면 어깨
        double elbow     = 0.0;   // θ4(R) / θ6(L): 팔꿈치 (스틱 방향각 보정 포함)
        bool success = false;
    };

    // 해당 팔의 어깨·팔꿈치·손목 관절 한계까지 검사한다. 허리(θ0) 한계는 검사하지 않는다.
    ArmIKResult solve_arm_ik(
        const std::array<double, 3>& p,
        double theta0,
        double theta_wrist,
        ArmSide side,
        bool print_err
    ) const;

    struct FKResult {
        std::array<double, 3> pR;   // 오른손 끝 좌표 (드럼 스틱 끝)
        std::array<double, 3> pL;   // 왼손 끝 좌표
        bool success = false;
    };
    
    FKResult solve_fk(const std::array<double, 9>& q);

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

    void verify_fk_ik(int num_tests = 1000, double tolerance_deg = 0.01);

    double get_effective_length(double theta_wrist) const;  // 하완+스틱 합성 링크 길이
    double get_effective_theta(double theta_wrist) const;   // 하완+스틱 합성 링크 방향각

    std::array<std::array<double, 4>, 4> dh_transform(double a, double alpha, double d, double theta);
    void mat4_mul_inplace(std::array<std::array<double, 4>, 4>& A, const std::array<std::array<double, 4>, 4>& B);
};