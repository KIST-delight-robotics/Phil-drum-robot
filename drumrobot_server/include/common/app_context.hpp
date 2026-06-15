#pragma once

#include <atomic>

enum class RobotState { STANDBY, INIT, IDLE, PLAYING, SHUTTINGDOWN };

struct AppContext {
    std::atomic<bool> running{true};                // 전체 종료 플래그 (false 되면 모든 스레드 루프 탈출)
    std::atomic<bool> send_active{false};           // send_loop 활성화 신호
    std::atomic<bool> recv_active{false};           // recv_loop 활성화 신호

    std::atomic<RobotState> robot_state{RobotState::STANDBY};  // 로봇 상태

    std::atomic<bool> play_abort{false};            // play 중 중단 플래그
};