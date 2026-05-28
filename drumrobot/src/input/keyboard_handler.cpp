#include "input/keyboard_handler.hpp"

KeyboardHandler::KeyboardHandler(AppContext &ctxRef, CommandQueue &commandQueueRef)
    : ctx(ctxRef), command_queue(commandQueueRef) {}

KeyboardHandler::~KeyboardHandler() {}

void KeyboardHandler::run() {
    std::string input;

    while (ctx.running.load()) {
        if (quitting) {
            usleep(100);    // 종료 대기
            continue;
        }

        if (!std::getline(std::cin, input)) {
            // EOF (Ctrl+D) 도 종료로 처리
            break;
        }

        // 명령을 CommandQueue 에 push
        if (!input.empty()) {
            command_queue.push(input);  // TODO: 패킷 구분자 포함해서 형식 맞춰서 보내기
        }

        // quit 비교는 대문자로 정규화해서
        std::string upper = input;
        std::transform(upper.begin(), upper.end(), upper.begin(),
                    [](unsigned char c){ return std::toupper(c); });
        if (upper == "QUIT" || upper == "Q") {
            quitting = true;
        }
    }

    ctx.running = false;
    std::cout << "[KeyboardHandler] 스레드 종료\n";
}