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

struct ControlData {
    std::vector<double> q;
    std::vector<double> qd;
    std::vector<ControlMode> mode;

    ControlData() = default;
    explicit ControlData(int n) 
        : q(n, 0.0), qd(n, 0.0), mode(n, ControlMode::None) {}
};

class ControlQueue {
public:
    ControlQueue();
    ~ControlQueue();

    void push(ControlData cmd);
    ControlData pop();
    bool empty();
    size_t size();

private:
    std::queue<ControlData> queue_;
    std::mutex mutex_;
};