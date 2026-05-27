#include "trajectory/behavior_planner.hpp"

BehaviorPlanner::BehaviorPlanner(AppContext &ctxRef) 
    : ctx(ctxRef) {}

BehaviorPlanner::~BehaviorPlanner() {

}

std::vector<MotionPrimitive> BehaviorPlanner::generate_motion_sequence(const ParsedCommand& parsed) {
    if(!parsed.valid) return;
    
    Opcode opcode = parsed.opcode;

    if (!ctx.send_active.load()) {
        // send_active 전에는 시작/종료 명령만 처리
        if (opcode == Opcode::START) {
            // 모션을 채우고 컨트롤 큐를 채운 후 send_active 켜기
        } else if (opcode == Opcode::QUIT) {
            ctx.shutdown_requested = true;
        } else {
            std::cout << "[BehaviorPlanner] 수행할 수 없는 명령\n";     // TODO: 어떤 명령을 왜 수행할 수 없는지 에러 메세지 추가
        }
    } else {
        
    }
}