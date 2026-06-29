#pragma once

#include <atomic>

enum class RobotState { STANDBY, INIT, IDLE, PLAYING, SHUTTINGDOWN };

struct AppContext {
    std::atomic<bool> running{true};                // 전체 종료 플래그 (false 되면 모든 스레드 루프 탈출)
    std::atomic<bool> send_active{false};           // send_loop 활성화 신호
    std::atomic<bool> recv_active{false};           // recv_loop 활성화 신호

    std::atomic<RobotState> robot_state{RobotState::STANDBY};  // 로봇 상태

    std::mutex last_q_mutex;                        // 마지막 목표 관절각 스냅샷 (BehaviorPlanner가 갱신, TcpServer가 조회)
    std::vector<double> last_q_target_snapshot;     // NOTE: 실측값이 아니라 "마지막으로 명령된 목표 자세"임

    std::atomic<bool> play_abort{false};            // play 중 중단 플래그
};