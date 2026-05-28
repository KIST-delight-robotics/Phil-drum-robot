#pragma once

#include <queue>
#include <mutex>
#include <optional>

enum class MotionType { TRANSLATE, IDLE };
enum class TrajectorySpace { JOINT, TASK };
enum class TrajectoryProfile { TRAPEZOIDAL, CUBIC, QUINTIC, COSINE };

struct MotionPrimitive {
    MotionType type = MotionType::TRANSLATE;
    
    // TRANSLATE용
    TrajectorySpace space = TrajectorySpace::JOINT;
    TrajectoryProfile profile = TrajectoryProfile::COSINE;
    std::vector<double> q_target;       // joint space일 때
    std::vector<double> p_target_R;     // task space일 때
    std::vector<double> p_target_L;
    double t_total = 4.0;
    
    // 연주용
    // ... 별도 필드
};

class MotionQueue {
public:
    MotionQueue();
    ~MotionQueue();

    void push(const MotionPrimitive& motion);
    bool empty();
    std::optional<MotionPrimitive> try_pop();

private:
    std::queue<MotionPrimitive> queue_;
    std::mutex mutex_;
};