#pragma once

#include <queue>
#include <mutex>

enum class MotionType {
    TRAPEZOIDAL,
    QUINTIC,
    DRUM_HIT,
};

struct MotionPrimitive {
    MotionType type;
    std::vector<double> q1;
    double t_total;
};

class MotionQueue {
public:
    MotionQueue();
    ~MotionQueue();

    void push(MotionPrimitive cmd);
    MotionPrimitive pop();
    bool empty();

private:
    std::queue<MotionPrimitive> queue_;
    std::mutex mutex_;
};