#pragma once

#include <atomic>

struct AppContext {
    std::atomic<bool> running{true};                // 전체 종료 플래그 (false 되면 모든 스레드 루프 탈출)
    std::atomic<bool> send_active{false};           // send_loop 활성화 신호
    std::atomic<bool> recv_active{false};           // recv_loop 활성화 신호
    std::atomic<bool> shutdown_requested{false};    // 새 명령 안 받겠다는 신호
};