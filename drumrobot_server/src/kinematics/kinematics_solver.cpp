#include "kinematics/kinematics_solver.hpp"

KinematicsSolver::KinematicsSolver() {}
KinematicsSolver::~KinematicsSolver() {}

void KinematicsSolver::initialize() {
    using json = nlohmann::json;

    std::ifstream f("drumrobot_server/config/kinematics.json");
    if (!f.is_open()) {
        std::cerr << "[KinematicsSolver] Failed to open config/kinematics.json\n";
        return;
    }
    json config = json::parse(f);

    for (auto& j : config["joint_limits"]) {
        int id = j["joint"];
        joint_limits[id] = {
            j["min_angle"].get<double>() * M_PI / 180.0,
            j["max_angle"].get<double>() * M_PI / 180.0
        };
    }

    link_length.waist     = config["link_length"]["waist"];
    link_length.upper_arm = config["link_length"]["upper_arm"];
    link_length.forearm   = config["link_length"]["forearm"];
    link_length.stick     = config["link_length"]["stick"];

    // std::cout << "[KinematicsSolver] Initialized."
    //           << " waist="     << link_length.waist
    //           << " upper_arm=" << link_length.upper_arm
    //           << " forearm="   << link_length.forearm
    //           << " stick="     << link_length.stick << "\n";

    // 기구학 함수 검증
    // verify_fk_ik();
}

KinematicsSolver::IKResult KinematicsSolver::solve_ik(
    const std::array<double, 3>& pR,
    const std::array<double, 3>& pL,
    double theta0,
    double theta7,
    double theta8,
    bool print_err
) const {
    IKResult result;
    result.q.fill(0.0);

    // 양팔 각각 해석 (팔별 어깨·팔꿈치·손목 한계 포함) 후 허리(θ0) 한계 검사
    ArmIKResult right = solve_arm_ik(pR, theta0, theta7, ArmSide::RIGHT, print_err);
    if (!right.success) return result;

    ArmIKResult left = solve_arm_ik(pL, theta0, theta8, ArmSide::LEFT, print_err);
    if (!left.success) return result;

    if (!check_joint_limit(0, theta0, print_err)) return result;

    // ----- 결과 적재 -----
    result.q = { theta0, right.shoulder1, left.shoulder1, right.shoulder2, right.elbow,
                 left.shoulder2, left.elbow, theta7, theta8 };
    result.success = true;
    return result;
}

KinematicsSolver::ArmIKResult KinematicsSolver::solve_arm_ik(
    const std::array<double, 3>& p,
    double theta0,
    double theta_wrist,
    ArmSide side,
    bool print_err
) const {
    ArmIKResult result;

    const bool   is_right = (side == ArmSide::RIGHT);
    const char*  label    = is_right ? "Right" : "Left";
    const double L1 = link_length.upper_arm;
    const double S  = link_length.waist;
    const double L2 = get_effective_length(theta_wrist);

    // ----- 어깨 위치 (오른팔 +0.5S, 왼팔 -0.5S) -----
    const double half_S    = is_right ? 0.5 * S : -0.5 * S;
    const double shoulderX = half_S * std::cos(theta0);
    const double shoulderY = half_S * std::sin(theta0);

    // ----- 수평 회전 (θ1 / θ2) -----
    double shoulder1 = std::atan2(p[1] - shoulderY, p[0] - shoulderX) - theta0;

    // ----- 수직 평면 (θ3,θ4 / θ5,θ6) -----
    double zeta = -p[2];
    double r2   = (p[1] - shoulderY) * (p[1] - shoulderY)
                + (p[0] - shoulderX) * (p[0] - shoulderX);

    double x   = zeta * zeta + r2 - L1 * L1 - L2 * L2;
    double rad = 4.0 * L1 * L1 * L2 * L2 - x * x;

    if (rad < 0.0) {
        if (print_err) std::cerr << "[KinematicsSolver] " << label << " arm unreachable (rad < 0)\n";
        return result;
    }

    double elbow      = std::atan2(std::sqrt(rad), x);
    double shoulder12 = std::atan2(std::sqrt(std::max(r2, 0.0)), zeta);
    double shoulder2  = shoulder12
                      - std::atan2(L2 * std::sin(elbow),
                                   L1 + L2 * std::cos(elbow));

    // 스틱 방향각 보정
    elbow -= get_effective_theta(theta_wrist);

    // NaN / Inf 체크
    for (double v : {shoulder1, shoulder2, elbow}) {
        if (std::isnan(v) || std::isinf(v)) {
            if (print_err) std::cerr << "[KinematicsSolver] NaN/Inf in " << label << " arm result\n";
            return result;
        }
    }

    // 관절 한계 검사 (어깨1, 어깨2, 팔꿈치, 손목). 허리(0)는 호출 측에서 검사
    if (!check_joint_limit(is_right ? 1 : 2, shoulder1,   print_err) ||
        !check_joint_limit(is_right ? 3 : 5, shoulder2,   print_err) ||
        !check_joint_limit(is_right ? 4 : 6, elbow,       print_err) ||
        !check_joint_limit(is_right ? 7 : 8, theta_wrist, print_err)) {
        return result;
    }

    result.shoulder1 = shoulder1;
    result.shoulder2 = shoulder2;
    result.elbow     = elbow;
    result.success   = true;
    return result;
}

KinematicsSolver::FKResult KinematicsSolver::solve_fk(const std::array<double, 9>& q) {
    FKResult result;
 
    if (q.size() < 9) {
        std::cerr << "[KinematicsSolver] forward(): q size < 9 ("
                  << q.size() << ")\n";
        return result;
    }
 
    // 관절 매핑
    const double Q1 = q[0];   // waist
    const double Q2 = q[1];   // right_shoulder_1
    const double Q3 = q[2];   // left_shoulder_1
    const double Q4 = q[3];   // right_shoulder_2
    const double Q5 = q[4];   // right_elbow
    const double Q6 = q[5];   // left_shoulder_2
    const double Q7 = q[6];   // left_elbow
    const double Q8 = q[7];   // right_wrist
    const double Q9 = q[8];   // left_wrist
 
    // 링크 파라미터
    const double z0    = 0.0;                       // 베이스 높이 (kinematics.json에 z0 없음 — 필요 시 추가)
    const double s     = link_length.waist;         // 어깨 간격
    const double r1    = link_length.upper_arm;     // 오른팔 상완
    const double r2    = link_length.forearm;       // 오른팔 하완
    const double L1    = link_length.upper_arm;     // 왼팔 상완 (좌우 동일 가정)
    const double L2    = link_length.forearm;       // 왼팔 하완
    const double stick = link_length.stick;
 
    // ===== 오른팔 FK =====
    // MATLAB:
    //   DH = [0    0       z0   Q(1);
    //         0    0.5*s   0    Q(2);
    //         pi/2 0       0    Q(4)-pi/2;
    //         0    r1      0    Q(5);
    //         0    r2      0    Q(8);
    //         0    stick   0    0     ];
    {
        const double dh_R[6][4] = {
            { 0.0,         0.0,      z0,    Q1            },
            { 0.0,         0.5 * s,  0.0,   Q2            },
            { M_PI / 2.0,  0.0,      0.0,   Q4 - M_PI / 2.0 },
            { 0.0,         r1,       0.0,   Q5            },
            { 0.0,         r2,       0.0,   Q8            },
            { 0.0,         stick,    0.0,   0.0           }
        };
 
        std::array<std::array<double, 4>, 4> TR = {{
            {1.0, 0.0, 0.0, 0.0},
            {0.0, 1.0, 0.0, 0.0},
            {0.0, 0.0, 1.0, 0.0},
            {0.0, 0.0, 0.0, 1.0}
        }};
 
        for (int i = 0; i < 6; ++i) {
            const double alpha = dh_R[i][0];
            const double a     = dh_R[i][1];
            const double d     = dh_R[i][2];
            const double theta = dh_R[i][3];
            auto Ti = dh_transform(a, alpha, d, theta);
            mat4_mul_inplace(TR, Ti);
        }
 
        result.pR[0] = TR[0][3];
        result.pR[1] = TR[1][3];
        result.pR[2] = TR[2][3];
    }
 
    // ===== 왼팔 FK =====
    // MATLAB:
    //   DH = [0    0        z0   Q(1);
    //         0    -0.5*s   0    Q(3);
    //         pi/2 0        0    Q(6)-pi/2;
    //         0    L1       0    Q(7);
    //         0    L2       0    Q(9);
    //         0    stick    0    0     ];
    {
        const double dh_L[6][4] = {
            { 0.0,         0.0,       z0,    Q1            },
            { 0.0,        -0.5 * s,   0.0,   Q3            },
            { M_PI / 2.0,  0.0,       0.0,   Q6 - M_PI / 2.0 },
            { 0.0,         L1,        0.0,   Q7            },
            { 0.0,         L2,        0.0,   Q9            },
            { 0.0,         stick,     0.0,   0.0           }
        };
 
        std::array<std::array<double, 4>, 4> TL = {{
            {1.0, 0.0, 0.0, 0.0},
            {0.0, 1.0, 0.0, 0.0},
            {0.0, 0.0, 1.0, 0.0},
            {0.0, 0.0, 0.0, 1.0}
        }};
 
        for (int i = 0; i < 6; ++i) {
            const double alpha = dh_L[i][0];
            const double a     = dh_L[i][1];
            const double d     = dh_L[i][2];
            const double theta = dh_L[i][3];
            auto Ti = dh_transform(a, alpha, d, theta);
            mat4_mul_inplace(TL, Ti);
        }
 
        result.pL[0] = TL[0][3];
        result.pL[1] = TL[1][3];
        result.pL[2] = TL[2][3];
    }
 
    // NaN / Inf 체크
    for (int i = 0; i < 3; ++i) {
        if (std::isnan(result.pR[i]) || std::isinf(result.pR[i]) ||
            std::isnan(result.pL[i]) || std::isinf(result.pL[i])) {
            std::cerr << "[KinematicsSolver] forward(): NaN/Inf in result\n";
            return result;
        }
    }
 
    result.success = true;
    return result;
}

void KinematicsSolver::verify_fk_ik(int num_tests, double tolerance_deg) {
    if (joint_limits.size() < 9) {
        std::cerr << "[KinematicsSolver] verify_fk_ik(): joint_limits 미초기화\n";
        return;
    }
 
    std::mt19937 rng(std::random_device{}());
 
    std::array<std::uniform_real_distribution<double>, 9> dists;
    for (int i = 0; i < 9; ++i) {
        auto it = joint_limits.find(i);
        if (it == joint_limits.end()) {
            std::cerr << "[KinematicsSolver] verify_fk_ik(): joint " << i
                      << " 한계 없음\n";
            return;
        }
        dists[i] = std::uniform_real_distribution<double>(
            it->second.min_angle,
            it->second.max_angle
        );
    }
 
    const double tolerance_rad = tolerance_deg * M_PI / 180.0;
 
    int pass = 0;
    int fk_fail = 0;
    int ik_fail = 0;
    int mismatch_same_endpoint = 0;   // 관절각은 다른데 손끝 위치 같음 (IK 다중해)
    int mismatch_diff_endpoint = 0;   // 손끝 위치도 다름 (공식 버그 가능성)
 
    // 최악 사례 추적
    double max_endpoint_err = 0.0;
    std::array<double, 9> worst_q_in{};
    std::array<double, 9> worst_q_out{};
    std::array<double, 3> worst_pR_target{};
    std::array<double, 3> worst_pR_actual{};
    std::array<double, 3> worst_pL_target{};
    std::array<double, 3> worst_pL_actual{};
 
    for (int t = 0; t < num_tests; ++t) {
        std::array<double, 9> q_in{};
        for (int i = 0; i < 9; ++i) {
            q_in[i] = dists[i](rng);
        }
 
        // FK: q_in → (pR, pL)
        FKResult fk = solve_fk(q_in);
        if (!fk.success) {
            fk_fail++;
            continue;
        }
 
        // IK: (pR, pL, theta0, theta7, theta8) → q_out
        IKResult ik = solve_ik(fk.pR, fk.pL, q_in[0], q_in[7], q_in[8], false);
        if (!ik.success) {
            ik_fail++;
            continue;
        }
 
        // 관절각 비교
        bool joint_match = true;
        for (int i = 0; i < 9; ++i) {
            double err = std::abs(q_in[i] - ik.q[i]);
            err = std::fmod(err, 2.0 * M_PI);
            if (err > M_PI) err = 2.0 * M_PI - err;
            if (err > tolerance_rad) {
                joint_match = false;
                break;
            }
        }
 
        if (joint_match) {
            pass++;
            continue;
        }
 
        // 관절각 mismatch → IK 결과를 다시 FK로 검증
        FKResult fk2 = solve_fk(ik.q);
        if (!fk2.success) {
            mismatch_diff_endpoint++;
            continue;
        }
 
        double dxR = fk.pR[0] - fk2.pR[0];
        double dyR = fk.pR[1] - fk2.pR[1];
        double dzR = fk.pR[2] - fk2.pR[2];
        double dxL = fk.pL[0] - fk2.pL[0];
        double dyL = fk.pL[1] - fk2.pL[1];
        double dzL = fk.pL[2] - fk2.pL[2];
 
        double endpoint_err_R = std::sqrt(dxR*dxR + dyR*dyR + dzR*dzR);
        double endpoint_err_L = std::sqrt(dxL*dxL + dyL*dyL + dzL*dzL);
        double endpoint_err = std::max(endpoint_err_R, endpoint_err_L);
 
        const double ENDPOINT_TOL = 1e-4;   // 0.1 mm
        if (endpoint_err < ENDPOINT_TOL) {
            mismatch_same_endpoint++;
        } else {
            mismatch_diff_endpoint++;
 
            if (endpoint_err > max_endpoint_err) {
                max_endpoint_err = endpoint_err;
                worst_q_in = q_in;
                worst_q_out = ik.q;
                worst_pR_target = fk.pR;
                worst_pR_actual = fk2.pR;
                worst_pL_target = fk.pL;
                worst_pL_actual = fk2.pL;
            }
        }
    }
 
    // ===== 결과 출력 =====
    std::cout << "\n========== FK-IK Round-trip Verification ==========\n";
    std::cout << "Total tests              : " << num_tests << "\n";
    std::cout << "  Pass (joint match)     : " << pass << "\n";
    std::cout << "  Mismatch (same endpt)  : " << mismatch_same_endpoint
              << "   <- IK multi-solution, OK\n";
    std::cout << "  Mismatch (diff endpt)  : " << mismatch_diff_endpoint
              << "   <- formula bug suspected\n";
    std::cout << "  FK failed              : " << fk_fail << "\n";
    std::cout << "  IK failed              : " << ik_fail << "\n";
    std::cout << "Joint tolerance          : " << tolerance_deg << " deg\n";
    std::cout << "Endpoint tolerance       : 0.1 mm\n";
 
    if (mismatch_diff_endpoint > 0) {
        std::cout << "\n[Worst case — endpoint mismatch]\n";
        std::cout << std::fixed << std::setprecision(4);
        std::cout << "  Max endpoint error     : "
                  << max_endpoint_err * 1000.0 << " mm\n";
        std::cout << "  q_in  [deg]: ";
        for (int i = 0; i < 9; ++i) {
            std::cout << std::setw(9) << worst_q_in[i] * 180.0 / M_PI;
            if (i < 8) std::cout << ",";
        }
        std::cout << "\n";
        std::cout << "  q_out [deg]: ";
        for (int i = 0; i < 9; ++i) {
            std::cout << std::setw(9) << worst_q_out[i] * 180.0 / M_PI;
            if (i < 8) std::cout << ",";
        }
        std::cout << "\n";
        std::cout << "  pR target  [m]: ("
                  << worst_pR_target[0] << ", "
                  << worst_pR_target[1] << ", "
                  << worst_pR_target[2] << ")\n";
        std::cout << "  pR actual  [m]: ("
                  << worst_pR_actual[0] << ", "
                  << worst_pR_actual[1] << ", "
                  << worst_pR_actual[2] << ")\n";
        std::cout << "  pL target  [m]: ("
                  << worst_pL_target[0] << ", "
                  << worst_pL_target[1] << ", "
                  << worst_pL_target[2] << ")\n";
        std::cout << "  pL actual  [m]: ("
                  << worst_pL_actual[0] << ", "
                  << worst_pL_actual[1] << ", "
                  << worst_pL_actual[2] << ")\n";
    }
    std::cout << "===================================================\n\n";
}

bool KinematicsSolver::check_joint_limits(const std::array<double, 9>& q, bool print_err) const {
    for (int i = 0; i < static_cast<int>(q.size()); ++i) {
        if (!check_joint_limit(i, q[i], print_err)) return false;
    }
    return true;
}

bool KinematicsSolver::check_joint_limit(int joint, double value, bool print_err) const {
    auto it = joint_limits.find(joint);
    if (it == joint_limits.end()) return true;   // 한계 미정의 관절은 통과

    if (value < it->second.min_angle || value > it->second.max_angle) {
        if (print_err) {
            std::cerr << "[KinematicsSolver] Joint " << joint
                      << " out of range: " << value * 180.0 / M_PI << " deg"
                      << "  (limit: "
                      << it->second.min_angle * 180.0 / M_PI << " ~ "
                      << it->second.max_angle * 180.0 / M_PI << " deg)\n";
        }
        return false;
    }
    return true;
}

double KinematicsSolver::get_effective_length(double theta_wrist) const {
    double x = link_length.forearm + link_length.stick * std::cos(theta_wrist);
    double y = link_length.stick   * std::sin(theta_wrist);
    return std::sqrt(x * x + y * y);
}

double KinematicsSolver::get_effective_theta(double theta_wrist) const {
    double x = link_length.forearm + link_length.stick * std::cos(theta_wrist);
    double y = link_length.stick   * std::sin(theta_wrist);
    return std::atan2(y, x);
}

std::array<std::array<double, 4>, 4> KinematicsSolver::dh_transform(double a, double alpha, double d, double theta) {
    // Craig (modified) DH 변환 행렬
    // T = [ ct,    -st,    0,      a;
    //       st*ca, ct*ca, -sa,    -d*sa;
    //       st*sa, ct*sa,  ca,     d*ca;
    //       0,     0,      0,      1 ]

    const double ca = std::cos(alpha);
    const double sa = std::sin(alpha);
    const double ct = std::cos(theta);
    const double st = std::sin(theta);
 
    std::array<std::array<double, 4>, 4> T = {{
        { ct,     -st,     0.0,    a       },
        { st*ca,   ct*ca, -sa,    -d*sa    },
        { st*sa,   ct*sa,  ca,     d*ca    },
        { 0.0,     0.0,    0.0,    1.0     }
    }};
    return T;
}

void KinematicsSolver::mat4_mul_inplace(std::array<std::array<double, 4>, 4>& A, const std::array<std::array<double, 4>, 4>& B) {
    // 4x4 행렬 곱 A = A * B

    std::array<std::array<double, 4>, 4> R{};
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            double sum = 0.0;
            for (int k = 0; k < 4; ++k) {
                sum += A[i][k] * B[k][j];
            }
            R[i][j] = sum;
        }
    }
    A = R;
}
