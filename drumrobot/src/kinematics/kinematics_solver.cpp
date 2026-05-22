#include "kinematics/kinematics_solver.hpp"

KinematicsSolver::KinematicsSolver() {}
KinematicsSolver::~KinematicsSolver() {}

void KinematicsSolver::initialize() {
    using json = nlohmann::json;

    std::ifstream f("drumrobot/config/kinematics.json");
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

    std::cout << "[KinematicsSolver] Initialized."
              << " waist="     << link_length.waist
              << " upper_arm=" << link_length.upper_arm
              << " forearm="   << link_length.forearm
              << " stick="     << link_length.stick << "\n";
}

KinematicsSolver::IKResult KinematicsSolver::solve(
    const std::array<double, 3>& pR,
    const std::array<double, 3>& pL,
    double theta0,
    double theta7,
    double theta8
) const {
    IKResult result;
    result.q.resize(9, 0.0);

    const double L1   = link_length.upper_arm;
    const double S    = link_length.waist;

    const double L2_R = get_effective_length(theta7);
    const double L2_L = get_effective_length(theta8);

    // ----- 어깨 위치 -----
    const double shoulderXR =  0.5 * S * std::cos(theta0);
    const double shoulderYR =  0.5 * S * std::sin(theta0);
    const double shoulderXL = -0.5 * S * std::cos(theta0);
    const double shoulderYL = -0.5 * S * std::sin(theta0);

    // ----- θ1 : right_shoulder_1 (수평) -----
    double theta1 = std::atan2(pR[1] - shoulderYR, pR[0] - shoulderXR) - theta0;

    // ----- θ2 : left_shoulder_1 (수평) -----
    double theta2 = std::atan2(pL[1] - shoulderYL, pL[0] - shoulderXL) - theta0;

    // ----- 오른팔 수직 평면 (θ3, θ4) -----
    double zeta_r = -pR[2];
    double r2_r   = (pR[1] - shoulderYR) * (pR[1] - shoulderYR)
                  + (pR[0] - shoulderXR) * (pR[0] - shoulderXR);

    double x_r   = zeta_r * zeta_r + r2_r - L1 * L1 - L2_R * L2_R;
    double rad_r  = 4.0 * L1 * L1 * L2_R * L2_R - x_r * x_r;

    if (rad_r < 0.0) {
        std::cerr << "[KinematicsSolver] Right arm unreachable (rad < 0)\n";
        return result;
    }

    double theta4  = std::atan2(std::sqrt(rad_r), x_r);
    double theta34 = std::atan2(std::sqrt(std::max(r2_r, 0.0)), zeta_r);
    double theta3  = theta34
                   - std::atan2(L2_R * std::sin(theta4),
                                L1   + L2_R * std::cos(theta4));

    // ----- 왼팔 수직 평면 (θ5, θ6) -----
    double zeta_l = -pL[2];
    double r2_l   = (pL[1] - shoulderYL) * (pL[1] - shoulderYL)
                  + (pL[0] - shoulderXL) * (pL[0] - shoulderXL);

    double x_l   = zeta_l * zeta_l + r2_l - L1 * L1 - L2_L * L2_L;
    double rad_l  = 4.0 * L1 * L1 * L2_L * L2_L - x_l * x_l;

    if (rad_l < 0.0) {
        std::cerr << "[KinematicsSolver] Left arm unreachable (rad < 0)\n";
        return result;
    }

    double theta6  = std::atan2(std::sqrt(rad_l), x_l);
    double theta56 = std::atan2(std::sqrt(std::max(r2_l, 0.0)), zeta_l);
    double theta5  = theta56
                   - std::atan2(L2_L * std::sin(theta6),
                                L1   + L2_L * std::cos(theta6));

    // 스틱 방향각 보정
    theta4 -= get_effective_theta(theta7);
    theta6 -= get_effective_theta(theta8);

    // ----- 결과 적재 -----
    result.q = { theta0, theta1, theta2, theta3, theta4,
                 theta5, theta6, theta7, theta8 };

    // NaN / Inf 체크
    for (double v : result.q) {
        if (std::isnan(v) || std::isinf(v)) {
            std::cerr << "[KinematicsSolver] NaN/Inf in result\n";
            return result;
        }
    }

    // 관절 한계 검사
    if (!check_joint_limits(result.q)) {
        return result;
    }

    result.success = true;
    return result;
}

bool KinematicsSolver::check_joint_limits(const std::vector<double>& q) const {
    for (int i = 0; i < static_cast<int>(q.size()); ++i) {
        auto it = joint_limits.find(i);
        if (it == joint_limits.end()) continue;

        if (q[i] < it->second.min_angle || q[i] > it->second.max_angle) {
            std::cerr << "[KinematicsSolver] Joint " << i
                      << " out of range: " << q[i] * 180.0 / M_PI << " deg"
                      << "  (limit: "
                      << it->second.min_angle * 180.0 / M_PI << " ~ "
                      << it->second.max_angle * 180.0 / M_PI << " deg)\n";
            return false;
        }
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