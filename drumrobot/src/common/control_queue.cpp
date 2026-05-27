#include "common/control_queue.hpp"

ControlQueue::ControlQueue() {}

ControlQueue::~ControlQueue() {}

void ControlQueue::push(const ControlSetPoint& point) {
    std::lock_guard<std::mutex> lock(mutex_);
    queue_.push(point);
}

ControlSetPoint ControlQueue::pop() {
    std::lock_guard<std::mutex> lock(mutex_);
    ControlSetPoint point = queue_.front();
    queue_.pop();
    return point;
}

bool ControlQueue::empty() {
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_.empty();
}

size_t ControlQueue::size() {
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_.size();
}