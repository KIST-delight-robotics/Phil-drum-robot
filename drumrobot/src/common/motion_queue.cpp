#include "common/motion_queue.hpp"

MotionQueue::MotionQueue() {}

MotionQueue::~MotionQueue() {}

void MotionQueue::push(MotionPrimitive data) {
    std::lock_guard<std::mutex> lock(mutex_);
    queue_.push(data);
}

MotionPrimitive MotionQueue::pop() {
    std::lock_guard<std::mutex> lock(mutex_);
    MotionPrimitive data = queue_.front();
    queue_.pop();
    return data;
}

bool MotionQueue::empty() {
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_.empty();
}
