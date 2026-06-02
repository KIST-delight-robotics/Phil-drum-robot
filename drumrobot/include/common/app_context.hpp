#pragma once

#include <atomic>

enum class RobotState { Init, Idle, Playing, ShuttingDown };

struct AppContext {
    std::atomic<bool> running{true};                // 전체 종료 플래그 (false 되면 모든 스레드 루프 탈출)
    std::atomic<bool> send_active{false};           // send_loop 활성화 신호
    std::atomic<bool> recv_active{false};           // recv_loop 활성화 신호

    std::atomic<RobotState> robot_state{RobotState::Init};  // 로봇 상태
};