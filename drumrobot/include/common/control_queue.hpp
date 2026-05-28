#pragma once

#include <queue>
#include <mutex>
#include <vector>
#include <optional>

enum class ControlMode {
    // T motor
    POS,
    VEL,
    
    // Maxon Motor
    CST,
    CSV,
    CSP,

    // Dynamixel or None
    None,
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

    void push(const ControlSetPoint& point);
    bool empty();
    size_t size();
    std::optional<ControlSetPoint> try_pop();

private:
    std::queue<ControlSetPoint> queue_;
    std::mutex mutex_;
};