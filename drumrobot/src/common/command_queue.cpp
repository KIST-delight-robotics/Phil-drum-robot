#include "common/command_queue.hpp"

CommandQueue::CommandQueue() {}

CommandQueue::~CommandQueue() {}

void CommandQueue::push(const std::string& cmd) {
    std::lock_guard<std::mutex> lock(mutex_);
    queue_.push(cmd);
}

bool CommandQueue::empty() {
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_.empty();
}

// std::string 타입 값이 있을수도 있고 없을 수도 있음
std::optional<std::string> CommandQueue::try_pop() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (queue_.empty()) return std::nullopt;
    std::string cmd = queue_.front();
    queue_.pop();
    return cmd;
}