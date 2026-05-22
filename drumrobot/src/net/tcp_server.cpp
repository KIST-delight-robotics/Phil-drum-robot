#include "net/tcp_server.hpp"

TcpServer::TcpServer(AppContext &ctxRef, int port, CommandQueue &commandQueueRef)
    : ctx(ctxRef), port_(port), server_fd_(-1), running_(false), command_queue(commandQueueRef) {}

TcpServer::~TcpServer() {
    
}

void TcpServer::run() {
    set_server();

    std::cout << "[TcpServer] Listening on port " << port_ << "\n";
    
    while (ctx.running.load()) {
        if (quitting) {
            usleep(100);    // 종료 대기
            continue;
        }

        sockaddr_in client_addr{};
        socklen_t client_len = sizeof(client_addr);

        int client_fd = accept(server_fd_, (sockaddr*)&client_addr, &client_len);   // client 연결될때까지 블로킹
        if (client_fd < 0) {
            if (ctx.running.load()) {
                std::cerr << "[TcpServer] accept() failed\n";
            }
            break;
        }

        std::cout << "[TcpServer] Client connected: "
                  << inet_ntoa(client_addr.sin_addr) << "\n";

        // handle client
        char buffer[1024];
        while (ctx.running.load()) {
            ssize_t bytes = recv(client_fd, buffer, sizeof(buffer) - 1, 0);
            if (bytes <= 0) {
                std::cout << "[TcpServer] Client disconnected.\n";
                break;
            }
            buffer[bytes] = '\0';

            std::string input(buffer);
            input.erase(input.find_last_not_of(" \r\n") + 1);  // trim

            std::cout << "[TcpServer] Received: " << input << "\n";

            // 명령을 CommandQueue 에 push
            if (!input.empty()) {
                command_queue.push(input);
            }

            if (input == "quit" || input == "q") {
                quitting = true;
                break;
            }
        }

        close(client_fd);
    }

    ctx.running = false;
    std::cout << "[TcpServer] 스레드 종료\n";
}

void TcpServer::stop() {
    if (server_fd_ >= 0) {
        close(server_fd_);
        server_fd_ = -1;
    }
}

void TcpServer::set_server() {
    server_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd_ < 0) {
        std::cerr << "[TcpServer] socket() failed" << std::endl;
        return;
    }

    int opt = 1;
    setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    sockaddr_in addr{};
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port        = htons(port_);

    if (bind(server_fd_, (sockaddr*)&addr, sizeof(addr)) < 0) {
        std::cerr << "[TcpServer] bind() failed" << std::endl;
        return;
    }

    if (listen(server_fd_, 5) < 0) {
        std::cerr << "[TcpServer] listen() failed" << std::endl;
        return;
    }
}
