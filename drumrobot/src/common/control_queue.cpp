#include "common/control_queue.hpp"

ControlQueue::ControlQueue() {}

ControlQueue::~ControlQueue() {}

void ControlQueue::push(ControlSetPoint data) {
    std::lock_guard<std::mutex> lock(mutex_);
    queue_.push(data);
}

ControlSetPoint ControlQueue::pop() {
    std::lock_guard<std::mutex> lock(mutex_);
    ControlSetPoint data = queue_.front();
    queue_.pop();
    return data;
}

bool ControlQueue::empty() {
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_.empty();
}

size_t ControlQueue::size() {
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_.size();
}