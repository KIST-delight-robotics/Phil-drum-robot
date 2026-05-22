#pragma once

#include <iostream>
#include <thread>
#include <pthread.h>
#include <string>
#include <map>
#include <vector>

#include "nlohmann/json.hpp"

#include "common/app_context.hpp"
#include "common/command_queue.hpp"
#include "common/control_queue.hpp"
#include "hardware/robot.hpp"
#include "kinematics/kinematics_solver.hpp"
 
enum class MotionType {
    TRAPEZOIDAL,
    QUINTIC,
    DRUM_HIT,
};

class MotionPlanner {
public:
    MotionPlanner(AppContext &ctxRef, CommandQueue &commandQueueRef, ControlQueue &controlQueueRef, Robot &robotRef);
    ~MotionPlanner();
 
    void run();
 
private:
    AppContext &ctx;
    CommandQueue &command_queue;
    ControlQueue &control_queue;

    Robot &robot;

    KinematicsSolver solver;

    std::map<std::string, std::vector<double>> poses;

    const long unsigned int threshold = 20;     // 궤적 생성 임계값
    const double dt = 0.005;                    // 데이터 시간 간격 5ms

    struct MotionRequest {
        MotionType type;
        std::vector<double> q1;
        double t_total;
    };

    std::queue<MotionRequest> motion_queue;

    std::vector<double> last_q;     // 마지막 위치
    std::vector<double> last_qd;    // 마지막 속도

    void initialize();
    void init_poses_from_json();
    
    void parse_command(const std::string &cmd);

    // ===== 모션 생성 함수 =====
    void generate_motion();
    
    void generate_trapezoidal_profile(std::vector<double> q0, std::vector<double> q1, double t_total);
};