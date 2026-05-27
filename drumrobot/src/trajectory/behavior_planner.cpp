#include "trajectory/behavior_planner.hpp"

BehaviorPlanner::BehaviorPlanner() {

}

BehaviorPlanner::~BehaviorPlanner() {

}

std::vector<MotionPrimitive> BehaviorPlanner::generate_motion_sequence(const ParsedCommand& parsed) {

}

// if (ctx.shutdown_requested.load()) return;

// 명령 파싱 및 motion_queue에 적재
// if (ctx.send_active) {
//     if (cmd == "home") {
//         motion_queue.push({MotionType::TRAPEZOIDAL, poses["home"], 4.0});
//     } else if (cmd == "ready") {
//         motion_queue.push({MotionType::TRAPEZOIDAL, poses["ready"], 4.0});
//     } else if (cmd == "stop") {
//         motion_queue.push({MotionType::TRAPEZOIDAL, last_q, 4.0});
//     } else if (cmd == "quit" || cmd == "q") {
//         motion_queue.push({MotionType::TRAPEZOIDAL, poses["shutdown"], 4.0});
//         ctx.shutdown_requested = true;
//     } else if (cmd == "idle") {
//         motion_queue.push({MotionType::TRAPEZOIDAL, last_q, 1.0});
//     } else {
//         std::cout << "[MotionPlanner] 알 수 없는 명령: " << cmd << "\n";
//     }
// } else {
//     // send_active 전에는 시작/종료 명령만 처리
//     if (cmd == "start") {
//         motion_queue.push({MotionType::TRAPEZOIDAL, poses["home"], 4.0});
//     } else if (cmd == "quit" || cmd == "q") {
//         ctx.shutdown_requested = true;
//     } else {
//         std::cout << "[MotionPlanner] 알 수 없는 명령: " << cmd << "\n";
//     }
// }