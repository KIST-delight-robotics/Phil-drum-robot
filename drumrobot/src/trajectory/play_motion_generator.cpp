#include "trajectory/play_motion_generator.hpp"

PlayMotionGenerator::PlayMotionGenerator() {

}

PlayMotionGenerator::~PlayMotionGenerator() {

}

void PlayMotionGenerator::initialize() {
    solver.initialize();

    base_motion_generator.initialize();
}

std::queue<std::array<double, ROBOT::NUM_JOINT>> PlayMotionGenerator::generate_motion(const std::vector<DrumEvent>& rds) {
    // rds[0]: 시작 자세
    // rds[1]: 목표 자세
    
    std::queue<std::array<double, ROBOT::NUM_JOINT>> q_queue;

    int n = get_num_point(rds[1].t, rds[0].t);

    std::queue<BaseMotionPoint> base_motion = base_motion_generator.generate_motion(rds, n);
    std::queue<HeadMotionPoint> head_motion = head_motion_generator.generate_motion(rds, n);
    std::queue<PedalMotionPoint> pedal_motion = pedal_motion_generator.generate_motion(rds, n);
    std::queue<StateMotionPoint> state_motion = state_motion_generator.generate_motion(rds, n);

    for (int i = 0; i < n; i++) {
        std::array<double, ROBOT::NUM_JOINT> q;

        BaseMotionPoint b = base_motion.front();
        base_motion.pop();

        HeadMotionPoint h = head_motion.front();
        head_motion.pop();

        PedalMotionPoint p = pedal_motion.front();
        pedal_motion.pop();

        StateMotionPoint s = state_motion.front();
        state_motion.pop();

        std::array<double, 3> pR = b.right_position;
        std::array<double, 3> pL = b.left_position;
        double theta0 = b.waist;
        double theta7 = b.right_wrist;
        double theta8 = b.left_wrist;
        KinematicsSolver::IKResult result = solver.ik_solve(pR, pL, theta0, theta7, theta8);

        if (!result.success) {
            std::cerr << "[PlayMotionGenerator] Failed to solve inverse kinematics\n";
            return q_queue; // TODO: 진행중인 동작을 멈춰야 함. 이렇게 중간에 끊고 다음거 이어서 만들면 계단 입력 나옴
        }

        for (int i = 0; i < 9; i++) {
            q[i] = result.q[i];   // 관절 0~8 (팔)
        }

        q[4] += s.right_elbow;
        q[6] += s.left_elbow;

        q[7] += s.right_wrist;
        q[8] += s.left_wrist;

        q[9] = p.right;
        q[10] = p.left;

        q[11] = h.yaw;
        q[12] = h.pitch;

        q_queue.push(q);
    }

    return q_queue;
}

int PlayMotionGenerator::get_num_point(double t0, double t1) {
    double n;

    // 한 라인의 데이터 개수 (5ms 단위)
    n = (t1 - t0) / ROBOT::DT_SECOND;
    round_sum += (int)(n * 10000) % 10000;
    if (round_sum >= 10000)
    {
        round_sum -= 10000;
        n++;
    }
    n = floor(n);

    return (int)n;
}