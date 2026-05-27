#include "common/motion_queue.hpp"

MotionQueue::MotionQueue() {}

MotionQueue::~MotionQueue() {}

void MotionQueue::push(const MotionPrimitive& motion) {
    std::lock_guard<std::mutex> lock(mutex_);
    queue_.push(motion);
}

MotionPrimitive MotionQueue::pop() {
    std::lock_guard<std::mutex> lock(mutex_);
    MotionPrimitive motion = queue_.front();
    queue_.pop();
    return motion;
}

bool MotionQueue::empty() {
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_.empty();
}
