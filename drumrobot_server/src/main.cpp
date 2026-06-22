#include <iostream>
#include <thread>
#include <pthread.h>
#include <map>
#include <string>
#include <memory>

// 오디오
#include "miniaudio/miniaudio.h"

#include "common/app_context.hpp"
#include "common/command_queue.hpp"
#include "common/control_queue.hpp"
#include "common/motion_queue.hpp"
#include "hardware/robot.hpp"
#include "tcp/tcp_server.hpp"
#include "realtime/controller.hpp"
#include "trajectory/motion_planner.hpp"

#define PORT 1951   // Phil Collins 출생연도

void set_priority(std::thread& t, int priority) {
    // 값이 클수록 더 높은 우선순위
    sched_param sch;
    sch.sched_priority = priority;
    if (pthread_setschedparam(t.native_handle(), SCHED_FIFO, &sch) != 0) {
        std::cerr << "Failed to set priority " << priority << std::endl;
    }
}

int main(int argc, char* argv[]) {
    // test 2
    ma_engine engine;
    ma_context context;

    ma_backend backends[] = { ma_backend_pulseaudio };   // pulse만
    ma_context_config cfg = ma_context_config_init();

    if (ma_context_init(backends, 1, &cfg, &context) != MA_SUCCESS) {
        std::cerr << "컨텍스트 초기화 실패\n";
        return -1;
    }
    std::cout << "선택된 백엔드: " << ma_get_backend_name(context.backend) << "\n";

    ma_engine_config ecfg = ma_engine_config_init();
    ecfg.pContext = &context;
    if (ma_engine_init(&ecfg, &engine) != MA_SUCCESS) {
        std::cerr << "엔진 초기화 실패\n";
        ma_context_uninit(&context);
        return -1;
    }

    std::cout << "재생 시작. Enter로 종료...\n";
    ma_engine_play_sound(&engine, "drumrobot_server/data/audio/TIM.wav", NULL);
    std::cin.get();

    ma_engine_uninit(&engine);
    ma_context_uninit(&context);
    return 0;

    AppContext ctx;
    CommandQueue command_queue;
    ControlQueue control_queue;
    MotionQueue motion_queue;

    Robot robot;
    robot.initialize();
    
    TcpServer server(ctx, PORT, command_queue);
    Controller controller(ctx, control_queue, robot);
    MotionPlanner motion_planner(ctx, command_queue, control_queue, motion_queue, robot);

    std::thread send_thread(&Controller::send_loop, &controller);
    std::thread recv_thread(&Controller::recv_loop, &controller);
    std::thread motion_planning_thread(&MotionPlanner::run, &motion_planner);
    std::thread tcp_server_thread(&TcpServer::run, &server);

    set_priority(send_thread, 40);
    set_priority(recv_thread, 30);
    set_priority(motion_planning_thread, 20);
    set_priority(tcp_server_thread, 10);

    send_thread.join();
    recv_thread.join();
    motion_planning_thread.join();
    tcp_server_thread.join();
    server.stop();

    return 0;
}