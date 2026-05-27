#pragma once

#include <queue>
#include <mutex>

enum class MotionType { TRANSLATE, IDLE };
enum class TrajectorySpace { JOINT, TASK };
enum class TrajectoryProfile { TRAPEZOIDAL, CUBIC, QUINTIC, COSINE };

struct MotionPrimitive {
    MotionType type;
    
    // TRANSLATE용
    TrajectorySpace space;
    TrajectoryProfile profile;
    std::vector<double> q_target;       // joint space일 때
    std::vector<double> p_target_R;     // task space일 때
    std::vector<double> p_target_L;
    double t_total;
    
    // 연주용
    // ... 별도 필드
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