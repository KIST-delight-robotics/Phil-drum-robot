#pragma once

#include <queue>
#include <mutex>
#include <vector>

enum class ControlMode {
    None,
    
    // T motor
    POS,
    VEL,
    
    // Maxon Motor
    CST,
    CSV,
    CSP,
};

struct ControlSetPoint {
    std::vector<double> q;
    std::vector<double> qd;
    std::vector<ControlMode> mode;

    ControlSetPoint() = default;
    explicit ControlSetPoint(int n) 
        : q(n, 0.0), qd(n, 0.0), mode(n, ControlMode::None) {}
};

class ControlQueue {
public:
    ControlQueue();
    ~ControlQueue();

    void push(ControlSetPoint cmd);
    ControlSetPoint pop();
    bool empty();
    size_t size();

private:
    std::queue<ControlSetPoint> queue_;
    std::mutex mutex_;
};