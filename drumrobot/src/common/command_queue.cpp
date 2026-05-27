#include "common/command_queue.hpp"

CommandQueue::CommandQueue() {}

CommandQueue::~CommandQueue() {}

void CommandQueue::push(const std::string& cmd) {
    std::lock_guard<std::mutex> lock(mutex_);
    queue_.push(cmd);
}

std::string CommandQueue::pop() {
    std::lock_guard<std::mutex> lock(mutex_);
    std::string cmd = queue_.front();
    queue_.pop();
    return cmd;
}

bool CommandQueue::empty() {
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_.empty();
}