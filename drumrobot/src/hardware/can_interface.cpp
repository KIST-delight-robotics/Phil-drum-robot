#include "hardware/can_interface.hpp"

CanInterface::CanInterface() {

}

CanInterface::~CanInterface() {

}

void CanInterface::initialize() {
    std::vector<std::string> ifnames = listAndActivateAvailableCANPorts();  // 사용 가능한 CAN 포트 이름 목록
    for (const auto &ifname : ifnames) {
        std::cout << "[CanInterface] Processing interface: " << ifname << std::endl;
        int hsocket = createSocket(ifname);
        if (hsocket < 0) {
            std::cerr << "[CanInterface] Socket creation error for interface: " << ifname << std::endl;
        } else {
            sockets[ifname] = hsocket;
            std::cout << "[CanInterface] Socket created for " << ifname << ": " << hsocket << std::endl;
        }
    }
}

bool CanInterface::sendFrame(const std::string &ifname, const can_frame &frame) {
    if (sockets.find(ifname) == sockets.end()) {
        std::cerr << "[CanInterface] Not connected: " << ifname << std::endl;
        return false;
    }

    ssize_t bytes = write(sockets[ifname], &frame, sizeof(struct can_frame));
    if (bytes != sizeof(struct can_frame)) {
        std::cerr << "[CanInterface] sendFrame() failed: " << ifname << std::endl;
        return false;
    }

    return true;
}

bool CanInterface::sendFrame(int socket, const can_frame &frame) {
    ssize_t bytes = write(socket, &frame, sizeof(struct can_frame));
    if (bytes != sizeof(struct can_frame)) {
        std::cerr << "[CanInterface] sendFrame() failed: socket " << socket << std::endl;
        return false;
    }
 
    return true;
}

bool CanInterface::receiveFrame(const std::string &ifname, can_frame &frame) {
    if (sockets.find(ifname) == sockets.end()) {
        std::cerr << "[CanInterface] Not connected: " << ifname << std::endl;
        return false;
    }

    ssize_t bytes = read(sockets[ifname], &frame, sizeof(struct can_frame));
    if (bytes != sizeof(struct can_frame)) {
        std::cerr << "[CanInterface] receiveFrame() failed: " << ifname << std::endl;
        return false;
    }

    return true;
}

bool CanInterface::receiveFrame(int socket, can_frame &frame) {
    ssize_t bytes = read(socket, &frame, sizeof(struct can_frame));
    if (bytes != sizeof(struct can_frame)) {
        std::cerr << "[CanInterface] receiveFrame() failed: socket " << socket << std::endl;
        return false;
    }
 
    return true;
}

bool CanInterface::sendandReceiveFrame(const std::string &ifname, can_frame &frame) {
    if (!sendFrame(ifname, frame) || !receiveFrame(ifname, frame)) {
        perror("[CanInterface] Send and receive error");
        return false;
    }
    return true;
}

bool CanInterface::sendandReceiveFrame(int socket, can_frame &frame) {
    if (!sendFrame(socket, frame) || !receiveFrame(socket, frame)) {
        perror("[CanInterface] Send and receive error");
        return false;
    }
    return true;
}

void CanInterface::setSocketNonBlock() {
    for (auto &socket : sockets) {
        int flags = fcntl(socket.second, F_GETFL, 0);
        if (flags < 0)
            continue;
        fcntl(socket.second, F_SETFL, flags | O_NONBLOCK); // 논블록 플래그 추가
    }
}

void CanInterface::setSocketBlock() {
    for (auto &socket : sockets) {
        int flags = fcntl(socket.second, F_GETFL, 0);
        if (flags < 0)
            continue;
        fcntl(socket.second, F_SETFL, flags & ~O_NONBLOCK); // 논블록 플래그 제거
    }
}

std::map<std::string, int> CanInterface::getSocket() {
    return sockets;
}

void CanInterface::setSocketsTimeout(int sec, int usec) {
    for (const auto &socketPair : sockets) {
        int socket_fd = socketPair.second;
        if (setSocketTimeout(socket_fd, sec, usec) != 0) {
            std::cerr << "[CanInterface] Failed to set socket timeout for " << socketPair.first << std::endl;
        }
    }
}

void CanInterface::clearReadBuffers() {
    for (const auto &socketPair : sockets) {
        int socket_fd = socketPair.second;
        clearCanBuffer(socket_fd);
    }
}

std::vector<std::string> CanInterface::listAndActivateAvailableCANPorts() {
    std::vector<std::string> ifnames;
    int portCount = 0;
 
    FILE *fp = popen("ip link show | grep can", "r");
    if (fp == nullptr) {
        perror("[CanInterface] No available CAN port");
        return ifnames;
    }
 
    char output[1024];
    while (fgets(output, sizeof(output) - 1, fp) != nullptr) {
        std::string line(output);
        std::istringstream iss(line);
        std::string skip, port;
        iss >> skip >> port;
 
        if (!port.empty() && port.back() == ':') {
            port.pop_back();
        }
 
        if (!port.empty() && port.find("can") == 0) {
            portCount++;
            if (!getCanPortStatus(port.c_str())) {
                printf("[CanInterface] %s is DOWN, activating...\n", port.c_str());
                activateCanPort(port.c_str());
            } else {
                printf("[CanInterface] %s is already UP\n", port.c_str());
            }
 
            ifnames.push_back(port);
        }
    }
 
    if (feof(fp) == 0) {
        perror("[CanInterface] fgets failed");
        printf("[CanInterface] Errno: %d\n", errno);
    }
 
    pclose(fp);
 
    if (portCount == 0) {
        printf("[CanInterface] No CAN port found.\n");
    }
 
    return ifnames;
}

bool CanInterface::getCanPortStatus(const char *port) {
    char command[50];
    snprintf(command, sizeof(command), "ip link show %s", port);

    FILE *fp = popen(command, "r");
    if (fp == NULL) {
        perror("[CanInterface] Error opening pipe");
        return false;
    }

    char output[1024];
    while (fgets(output, sizeof(output) - 1, fp) != NULL) {
        if (strstr(output, "DOWN") || strstr(output, "does not exist")) {
            pclose(fp);
            return false;
        } else if (strstr(output, "UP")) {
            pclose(fp);
            return true;
        }
    }

    pclose(fp);
    return false;
}

void CanInterface::activateCanPort(const char *port) {
    char command1[100], command2[100], command3[100], command4[100];

    snprintf(command4, sizeof(command4), "sudo ip link set %s down", port);
    snprintf(command1, sizeof(command1), "sudo ip link set %s type can bitrate 1000000 restart-ms 100", port);
    snprintf(command2, sizeof(command2), "sudo ip link set %s up", port);
    snprintf(command3, sizeof(command3), "sudo ifconfig %s txqueuelen 1000", port);

    int ret4 = system(command4);
    int ret1 = system(command1);
    int ret2 = system(command2);
    int ret3 = system(command3);

    if (ret1 != 0 || ret2 != 0 || ret3 != 0 || ret4 != 0) {
        fprintf(stderr, "[CanInterface] Failed to activate port: %s\n", port);
    }

    sleep(2);
}

int CanInterface::createSocket(const std::string& ifname) {
    int result;
    struct sockaddr_can addr;
    struct ifreq ifr;

    int localSocket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (localSocket < 0) {
        return ERR_SOCKET_CREATE_FAILURE;
    }

    memset(&ifr, 0, sizeof(struct ifreq));
    memset(&addr, 0, sizeof(struct sockaddr_can));

    strcpy(ifr.ifr_name, ifname.c_str());
    result = ioctl(localSocket, SIOCGIFINDEX, &ifr);
    if (result < 0) {
        close(localSocket);
        return ERR_SOCKET_CREATE_FAILURE;
    }

    addr.can_ifindex = ifr.ifr_ifindex;
    addr.can_family = AF_CAN;

    if (bind(localSocket, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        close(localSocket);
        return ERR_SOCKET_CREATE_FAILURE;
    }

    return localSocket;
}

int CanInterface::setSocketTimeout(int socket, int sec, int usec) {
    // non-blocking 모드 일때는 Timeout은 의미없음
    struct timeval timeout;
    timeout.tv_sec = sec;
    timeout.tv_usec = usec;

    if (setsockopt(socket, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout, sizeof(timeout)) < 0) {
        perror("[CanInterface] setsockopt failed");
        return -1;
    }

    return 0;
}

void CanInterface::clearCanBuffer(int canSocket) {
    struct can_frame frame;
    fd_set readSet;
    struct timeval timeout;

    // 수신 대기 시간 설정
    timeout.tv_sec = 0;
    timeout.tv_usec = 0; // 즉시 반환

    while (true) {
        FD_ZERO(&readSet);
        FD_SET(canSocket, &readSet);

        // 소켓에서 읽을 데이터가 있는지 확인
        int selectRes = select(canSocket + 1, &readSet, NULL, NULL, &timeout);
        // std::cout << "clearcanbuf" << std::endl;
        if (selectRes > 0) {
            // 수신 버퍼에서 데이터 읽기
            ssize_t nbytes = read(canSocket, &frame, sizeof(struct can_frame));
            if (nbytes < 0) {   // 읽기 실패
                if (errno == EWOULDBLOCK || errno == EAGAIN) {
                    // non-blocking 모드에서 읽을 데이터 없음 -> 루프 종료
                    break;
                } else {
                    perror("[CanInterface] read() failed while clearing CAN buffer");
                    break;
                }
            }
            else if (nbytes == 0) {
                // CAN 소켓이 닫힘 -> 루프 종료
                break;
            }
        } else {
            // 읽을 데이터 없음
            break;
        }
    }
}
