#include "hardware/robot.hpp"

Robot::Robot() {
    
}

Robot::~Robot() {
    
}

void Robot::initialize() {
    // TODO: CAN 초기화 하기 전 리셋 (USB 전원 껏다 켜기) 먼저 해야 함
    can.initialize();
    init_motor_from_json();
    set_motors_socket();
    maxon_motor_setting();
    can.setSocketNonBlock();
    set_zero_tmotor();
    maxon_motor_enable();
    set_maxon_motor_mode("CSP");
    init_dynamicxel();
    // USBIO
    // 아두이노

}

void Robot::init_motor_from_json() {
    using json = nlohmann::json;

    std::ifstream f("drumrobot/config/motors.json");
    if (!f.is_open()) {
        std::cerr << "[Robot] Failed to open config/motors.json\n";
        return;
    }

    json config = json::parse(f);

    for (auto &m : config["motors"]) {
        std::string type = m["type"];
        int id = m["id"];

        if (type == "TMotor") {
            auto motor = std::make_shared<TMotor>(id);
            motor->name = m["name"];
            motor->direction_sign = m["direction_sign"];
            motor->initial_joint_angle = m["initial_joint_angle"].get<double>() * M_PI / 180.0;
            motor->min_angle = m["min_angle"].get<double>() * M_PI / 180.0;
            motor->max_angle = m["max_angle"].get<double>() * M_PI / 180.0;
            motor->node_id = std::stoul(m["node_id"].get<std::string>(), nullptr, 16);
            motor->model = m["model"];
            motor->gear_ratio = m["gear_ratio"];
            motor->current_limit = m["current_limit"];
            motor->control_gain = m["control_gain"];
            motors[id] = motor;

            // std::cout << "[Robot] motor setting: " << motor->name << "\n";
        } else if (type == "MaxonMotor") {
            auto motor = std::make_shared<MaxonMotor>(id);
            motor->name = m["name"];
            motor->direction_sign = m["direction_sign"];
            motor->initial_joint_angle = m["initial_joint_angle"].get<double>() * M_PI / 180.0;
            motor->min_angle = m["min_angle"].get<double>() * M_PI / 180.0;
            motor->max_angle = m["max_angle"].get<double>() * M_PI / 180.0;
            motor->node_id = std::stoul(m["node_id"].get<std::string>(), nullptr, 16);
            motor->can_send_id = 0x600 + motor->node_id;
            motor->can_receive_id = 0x580 + motor->node_id;
            motor->tx_pdo_ids[0] = std::stoul(m["tx_pdo_ids[0]"].get<std::string>(), nullptr, 16);  // Controlword
            motor->tx_pdo_ids[1] = std::stoul(m["tx_pdo_ids[1]"].get<std::string>(), nullptr, 16);  // Target Position
            motor->tx_pdo_ids[2] = std::stoul(m["tx_pdo_ids[2]"].get<std::string>(), nullptr, 16);  // Target Velocity
            motor->tx_pdo_ids[3] = std::stoul(m["tx_pdo_ids[3]"].get<std::string>(), nullptr, 16);  // TargetTorque
            motor->rx_pdo_ids[0] = std::stoul(m["rx_pdo_ids[0]"].get<std::string>(), nullptr, 16);  // Statusword, Actual Position, Actual Torque
            motor->rx_pdo_ids[1] = std::stoul(m["rx_pdo_ids[1]"].get<std::string>(), nullptr, 16);  // ActualPosition, ActualTorque
            motor->control_kp = m["control_kp"];
            motor->control_kd = m["control_kd"];
            motors[id] = motor;

            // std::cout << "[Robot] motor setting: " << motor->name << "\n";
        } else if (type == "Dynamicxel") {
            auto motor = std::make_shared<DynamixelMotor>(id);
            motor->name = m["name"];
            motor->direction_sign = m["direction_sign"];
            motor->initial_joint_angle = m["initial_joint_angle"].get<double>() * M_PI / 180.0;
            motor->min_angle = m["min_angle"].get<double>() * M_PI / 180.0;
            motor->max_angle = m["max_angle"].get<double>() * M_PI / 180.0;
            motor->dxl_id = m["dxl_id"];
            motors[id] = motor;

            // std::cout << "[Robot] motor setting: " << motor->name << "\n";
        }
    }
}

void Robot::set_motors_socket() {
    struct can_frame frame;
    can.setSocketsTimeout(0, 10000);    // 타임아웃 10ms 설정
    can.clearReadBuffers();

    // 모든 소켓에 대해 연결을 확인
    std::map<std::string, int> sockets = can.getSocket();
    for (const auto &socket : sockets) {
        int socket_fd = socket.second;

        for (auto &[id, motor] : motors) {
            if (std::shared_ptr<TMotor> tmotor = std::dynamic_pointer_cast<TMotor>(motor)) {
                if (!tmotor->is_connected) {
                    tmotor->socket = socket_fd; // 임시로 소켓 설정
                }
            } else if (std::shared_ptr<MaxonMotor> maxon = std::dynamic_pointer_cast<MaxonMotor>(motor)) {
                if (!maxon->is_connected) {
                    maxon->socket = socket_fd;
                    m_codec.getCheck(maxon->can_send_id, &frame);
                    can.sendFrame(maxon->socket, frame);

                    usleep(50000);
                }
            }
        }

        // 해당 소켓으로 프레임 10개를 무작정 읽어서 버퍼에 넣음
        int readCount = 0;
        std::vector<can_frame> temp_frames;
        while (readCount < 10) {
            ssize_t result = read(socket_fd, &frame, sizeof(frame));

            if (result > 0) {
                temp_frames.push_back(frame);
            }
            readCount++;
        }

        // 버퍼에서 하나씩 읽어옴
        for (auto &[id, motor] : motors) {
            for (auto &frame : temp_frames) {
                if (std::shared_ptr<TMotor> tmotor = std::dynamic_pointer_cast<TMotor>(motor)) {
                    if ((frame.can_id & 0xFF) == tmotor->node_id) {
                        tmotor->is_connected = true;
                    }
                } else if (std::shared_ptr<MaxonMotor> maxon = std::dynamic_pointer_cast<MaxonMotor>(motor)) {
                    if (frame.can_id == maxon->can_receive_id) {
                        maxon->is_connected = true;
                    }
                }
            }
        }
    }

    // node_id 기준으로 정렬된 목록 생성
    std::vector<std::pair<uint32_t, int>> sorted_motors; // <node_id, map key>
    for (auto &[id, motor] : motors) {
        if (auto tmotor = std::dynamic_pointer_cast<TMotor>(motor)) {
            sorted_motors.push_back({tmotor->node_id, id});
        } else if (auto maxon = std::dynamic_pointer_cast<MaxonMotor>(motor)) {
            sorted_motors.push_back({maxon->node_id, id});
        }
    }
    std::sort(sorted_motors.begin(), sorted_motors.end());

    // 모터 연결 상태 확인 및 연결 안된 모터 삭제
    for (auto &[node_id, key] : sorted_motors) {
        auto it = motors.find(key);
        if (it == motors.end()) continue;

        std::shared_ptr<Motor> motor = it->second;

        if (auto tmotor = std::dynamic_pointer_cast<TMotor>(motor)) {
            if (tmotor->is_connected) {
                std::cout << "[Robot] --------------> CAN NODE ID " << node_id << " Connected. Joint " << tmotor->id << ": " << tmotor->name << "\n";
            } else {
                std::cerr << "[Robot] CAN NODE ID " << node_id << " Not Connected. Joint " << tmotor->id << ": " << tmotor->name << "\n";
                motors.erase(it);
            }
        } else if (auto maxon = std::dynamic_pointer_cast<MaxonMotor>(motor)) {
            if (maxon->is_connected) {
                std::cout << "[Robot] --------------> CAN NODE ID " << node_id << " Connected. Joint " << maxon->id << ": " << maxon->name << "\n";
            } else {
                std::cerr << "[Robot] CAN NODE ID " << node_id << " Not Connected. Joint " << maxon->id << ": " << maxon->name << "\n";
                motors.erase(it);
            }
        }
    }
}

void Robot::maxon_motor_setting() {
    // Count Maxon Motor Sockets
    for (auto &[id, motor] : motors) {
        auto maxon = std::dynamic_pointer_cast<MaxonMotor>(motor);
        if (!maxon) continue;

        if (virtual_maxon_motor.size() == 0) {
            virtual_maxon_motor.push_back(maxon);
        } else {
            bool otherSocket = true;
            int n = virtual_maxon_motor.size();
            for(int i = 0; i < n; i++) {
                if (virtual_maxon_motor[i]->socket == maxon->socket) {
                    otherSocket = false;
                }
            }

            if (otherSocket) {
                virtual_maxon_motor.push_back(maxon);
            }
        }
    }

    struct can_frame frame;
    can.setSocketsTimeout(2, 0);

    for (auto &[id, motor] : motors) {
        auto maxon = std::dynamic_pointer_cast<MaxonMotor>(motor);
        if (!maxon) continue;

        // CSP Settings
        m_codec.getCSPMode(maxon->can_send_id, &frame);
        can.sendandReceiveFrame(maxon->socket, frame);

        m_codec.getPosOffset(maxon->can_send_id, &frame);
        can.sendandReceiveFrame(maxon->socket, frame);

        m_codec.getTorqueOffset(maxon->can_send_id, &frame);
        can.sendandReceiveFrame(maxon->socket, frame);

        // CSV Settings
        m_codec.getCSVMode(maxon->can_send_id, &frame);
        can.sendandReceiveFrame(maxon->socket, frame);

        m_codec.getVelOffset(maxon->can_send_id, &frame);
        can.sendandReceiveFrame(maxon->socket, frame);

        // CST Settings
        m_codec.getCSTMode(maxon->can_send_id, &frame);
        can.sendandReceiveFrame(maxon->socket, frame);

        m_codec.getTorqueOffset(maxon->can_send_id, &frame);
        can.sendandReceiveFrame(maxon->socket, frame);

        // HMM Settings
        m_codec.getHomeMode(maxon->can_send_id, &frame);
        can.sendandReceiveFrame(maxon->socket, frame);

        m_codec.getHomingMethodR(maxon->can_send_id, &frame);
        can.sendandReceiveFrame(maxon->socket, frame);

        m_codec.getHomeoffsetDistance(maxon->can_send_id, &frame, 0);
        can.sendandReceiveFrame(maxon->socket, frame);

        m_codec.getHomePosition(maxon->can_send_id, &frame, 0);
        can.sendandReceiveFrame(maxon->socket, frame);

        // m_codec.getCurrentThresholdL(maxon->can_send_id, &frame);
        // can.sendandReceiveFrame(maxon->socket, frame);
    }
}

void Robot::set_zero_tmotor() {
    struct can_frame frame;

    for (auto &[id, motor] : motors) {
        auto tmotor = std::dynamic_pointer_cast<TMotor>(motor);
        if (!tmotor) continue;

        t_codec.setOrigin(tmotor->node_id, &frame, 0);
        can.sendFrame(tmotor->socket, frame);

        usleep(100000);    // 100ms
    }

    std::cout << "[Robot] TMotor Set Zero\n";
    sleep(2);   // Set Zero 명령이 확실히 실행된 후 종료
}

void Robot::maxon_motor_enable() {
    struct can_frame frame;
    can.setSocketsTimeout(2, 0);

    // 제어 모드 설정
    for (auto &[id, motor] : motors) {
        auto maxon = std::dynamic_pointer_cast<MaxonMotor>(motor);
        if (!maxon) continue;

        m_codec.getHomeMode(maxon->can_send_id, &frame);
        can.sendFrame(maxon->socket, frame);

        m_codec.getOperational(maxon->node_id, &frame);
        can.sendFrame(maxon->socket, frame);

        usleep(100000);

        m_codec.getShutdown(maxon->tx_pdo_ids[0], &frame);
        can.sendFrame(maxon->socket, frame);

        m_codec.getSync(&frame);
        can.sendFrame(maxon->socket, frame);

        usleep(100000);
        
        m_codec.getEnable(maxon->tx_pdo_ids[0], &frame);
        can.sendFrame(maxon->socket, frame);

        m_codec.getSync(&frame);
        can.sendFrame(maxon->socket, frame);

        usleep(100000);

        // m_codec.getStartHoming(maxon->tx_pdo_ids[0], &frame);
        // can.sendFrame(maxon->socket, frame);

        // m_codec.getSync(&frame);
        // can.sendFrame(maxon->socket, frame);

        // usleep(100000);
    }

    std::cout << "[Robot] Maxon Motor Enable\n";
}

void Robot::set_maxon_motor_mode(std::string targetMode) {
    struct can_frame frame;
    can.setSocketsTimeout(0, 10000);

    for (auto &[id, motor] : motors) {
        auto maxon = std::dynamic_pointer_cast<MaxonMotor>(motor);
        if (!maxon) continue;

        if (targetMode == "CSV") {          // Cyclic Sync Velocity Mode
            m_codec.getCSVMode(maxon->can_send_id, &frame);
            can.sendFrame(maxon->socket, frame);
        } else if (targetMode == "CST") {   // Cyclic Sync Torque Mode
            m_codec.getCSTMode(maxon->can_send_id, &frame);
            can.sendFrame(maxon->socket, frame);
        } else if (targetMode == "HMM") {   // Homming Mode
            m_codec.getHomeMode(maxon->can_send_id, &frame);
            can.sendFrame(maxon->socket, frame);
        } else if (targetMode == "CSP") {   // Cyclic Sync Position Mode
            m_codec.getCSPMode(maxon->can_send_id, &frame);
            can.sendFrame(maxon->socket, frame);
        }

        // 모드 바꾸고 껐다 켜주기

        m_codec.getShutdown(maxon->tx_pdo_ids[0], &frame);
        can.sendFrame(maxon->socket, frame);

        m_codec.getSync(&frame);
        can.sendFrame(maxon->socket, frame);

        usleep(100);

        m_codec.getEnable(maxon->tx_pdo_ids[0], &frame);
        can.sendFrame(maxon->socket, frame);

        m_codec.getSync(&frame);
        can.sendFrame(maxon->socket, frame);

        usleep(100);
    }

    std::cout << "[Robot] Maxon Motor Mode: " << targetMode << "\n";
}

void Robot::init_dynamicxel() {
    // TODO: USB 속도 설정인가? 그런거 해야 함
    dynamixel::PortHandler *port;
    dynamixel::PacketHandler *pkt;

    port = dynamixel::PortHandler::getPortHandler(DXL_PORT);
    pkt = dynamixel::PacketHandler::getPacketHandler(2.0);

    // Open port
    if (port->openPort())
    {
        printf("[Robot] -------------- Open Dynamixel Port");
    }
    else
    {
        printf("[Robot] Failed to open the Dynamixel port!\n");
        return;
    }

    // Set port baudrate
    if (port->setBaudRate(4500000))
    {
        printf("[Robot] -------------- change the baudrate!\n");
    }
    else
    {
        printf("\n[Robot] Failed to change the baudrate!\n");
        return;
    }

    // 연결 안된 모터
    std::vector<int> to_remove;

    uint8_t err = 0;
    for (auto &[id, motor] : motors) {
        auto dxl = std::dynamic_pointer_cast<DynamixelMotor>(motor);
        if (!dxl) continue;

        uint16_t dxl_model_number = 0;
        uint8_t dxl_error = 0;

        int dxl_comm_result = pkt->ping(port, dxl->dxl_id, &dxl_model_number, &dxl_error);

        if (dxl_comm_result == COMM_SUCCESS && dxl_error == 0) {
            // DXL 토크 ON
            pkt->write1ByteTxRx(port, dxl->dxl_id, 64, 1, &err);

            dxl->port = port;
            dxl->pkt = pkt;

            printf("[Robot] --------------> ID:%03d Found! Model number: %d\n", dxl->dxl_id, dxl_model_number);
        } else {
            printf("[Robot] ID:%03d COMM FAIL\n", dxl->dxl_id);
            to_remove.push_back(id);
        }
    }

    // 연결 안된 모터 삭제
    for (int id : to_remove) {
        motors.erase(id);
    }
}
