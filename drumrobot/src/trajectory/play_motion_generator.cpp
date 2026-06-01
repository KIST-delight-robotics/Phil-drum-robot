#include "trajectory/play_motion_generator.hpp"

PlayMotionGenerator::PlayMotionGenerator() {

}

PlayMotionGenerator::~PlayMotionGenerator() {

}

void PlayMotionGenerator::initialize() {
    solver.initialize();
}

std::queue<std::vector<double>> PlayMotionGenerator::generate_motion() {
    std::queue<std::vector<double>> q_queue;

    int n = get_num_point();

    std::queue<BaseMotionPoint> base_motion = base_motion_generator.generate_motion();
    std::queue<HeadMotionPoint> head_motion = head_motion_generator.generate_motion();
    std::queue<PedalMotionPoint> pedal_motion = pedal_motion_generator.generate_motion();
    std::queue<StateMotionPoint> state_motion = state_motion_generator.generate_motion();

    for (int i = 0; i < n; i++) {
        std::vector<double> q(NUM_JOINT);

        BaseMotionPoint b = base_motion.front();
        base_motion.pop();

        HeadMotionPoint h = head_motion.front();
        head_motion.pop();

        PedalMotionPoint p = pedal_motion.front();
        pedal_motion.pop();

        StateMotionPoint s = state_motion.front();
        state_motion.pop();

        std::array<double, 3> pR = {b.right_x, b.right_y, b.right_z};
        std::array<double, 3> pL = {b.left_x, b.left_y, b.left_z};
        double theta0 = b.waist;
        double theta7 = b.right_wrist;
        double theta8 = b.left_wrist;
        KinematicsSolver::IKResult result = solver.ik_solve(pR, pL, theta0, theta7, theta8);

        if (!result.success) {
            std::cerr << "[PlayMotionGenerator] Failed to solve inverse kinematics\n";
            return q_queue;
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

int PlayMotionGenerator::get_num_point() {
    return 100;
}