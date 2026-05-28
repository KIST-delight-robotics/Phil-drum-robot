#include "realtime/controller.hpp"

Controller::Controller(AppContext &ctxRef, ControlQueue &controlQueueRef, Robot &robotRef)
    : ctx(ctxRef), control_queue(controlQueueRef), robot(robotRef), motor_log("motor")
{
    int n = robot.num_joint;
    curr_point = ControlSetPoint(n);
    prev_point = ControlSetPoint(n);

    for (auto &[id, motor] : robot.motors) {
        if (id < n) {            
            curr_point.q[id] = motor->initial_joint_angle;
            prev_point.q[id] = motor->initial_joint_angle;
        }
    }

    std::vector<std::string> header = {"id", "mode", "desired", "actual", "err", "current/torque", "input"};
    motor_log.set_header(header);
}

Controller::~Controller() {}

void Controller::send_loop() {
    int cnt = 0;

    while (ctx.running.load()) {
        if (!ctx.send_active.load()) {
            if (ctx.shutdown_requested.load()) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(10)); // send_active 전까지 대기
            continue;
        }

        auto next = std::chrono::steady_clock::now();

        while (ctx.running.load() && ctx.send_active.load()) {
            next += std::chrono::microseconds(1000);    // 1ms 주기

            if (cnt == 0) {
                // 1ms: 큐에서 새 목표값 가져오고, 맥슨 보간 1번째 송신
                prev_point = curr_point;
                if (auto sp = control_queue.try_pop()) {
                    curr_point = *sp;
                } else {
                    static int err_cnt = 0;
                    if (err_cnt++ % 100 == 0) std::cerr << "[Controller] control_queue underflow\n";
                }

                send_task_1ms(cnt);
            } else if (cnt < 4) {
                // 2~4ms: 맥슨 보간 송신
                send_task_1ms(cnt);
            } else {
                // 5ms: TMotor + 맥슨값 동시 송신
                send_task_5ms();
                cnt = -1;               // 다음 루프에서 0이 됨
            }
            cnt++;

            // 종료 요청 + 큐 소진 -> 루프 탈출
            if (ctx.shutdown_requested.load() && control_queue.empty()) {
                ctx.running = false;
                break;
            }

            std::this_thread::sleep_until(next);
        }
    }

    ctx.running = false;
    std::cout << "[Controller] send_loop 종료\n";
}

void Controller::recv_loop() {
    while (ctx.running.load()) {
        if (!ctx.recv_active.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10)); // recv_active 전까지 대기
            continue;
        }

        auto next = std::chrono::steady_clock::now();

        while (ctx.running.load() && ctx.recv_active.load()) {
            next += std::chrono::microseconds(100);     // 100us 주기

            read_frames();
            distribute_frames();

            std::this_thread::sleep_until(next);
        }
    }

    ctx.running = false;
    std::cout << "[Controller] recv_loop 종료\n";
}

// ===== SEND =====
void Controller::send_task_1ms(int cnt) {
    double alpha = static_cast<double>(cnt + 1) / 5.0;

    ControlSetPoint interp(curr_point.q.size());
    interp.mode = curr_point.mode;
    for (size_t i = 0; i < curr_point.q.size(); ++i) {
        interp.q[i]    = prev_point.q[i] + alpha * (curr_point.q[i] - prev_point.q[i]);
    }
 
    maxon_motor_send_task(interp);

    // Sync 프레임: 소켓당 1회
    struct can_frame sync_frame;
    m_codec.getSync(&sync_frame);
    for (auto &maxon : robot.virtual_maxon_motor) {
        robot.can.sendFrame(maxon->socket, sync_frame);
    }
}

void Controller::send_task_5ms() {
    tmotor_send_task(curr_point);
    maxon_motor_send_task(curr_point);
    dynamicxel_send_task(curr_point);
 
    // Sync 프레임: 소켓당 1회
    struct can_frame sync_frame;
    m_codec.getSync(&sync_frame);
    for (auto &maxon : robot.virtual_maxon_motor) {
        robot.can.sendFrame(maxon->socket, sync_frame);
    }
}

void Controller::tmotor_send_task(const ControlSetPoint &point) {
    struct can_frame frame;
 
    for (auto &[id, motor] : robot.motors) {
        auto tmotor = std::dynamic_pointer_cast<TMotor>(motor);
        if (!tmotor) continue;

        ControlMode mode = point.mode[id];
        double motor_position  = tmotor->joint_angle_to_motor_position(point.q[id]);
        double motor_velocity = tmotor->direction_sign * point.qd[id];    // rad/s
 
        // 목표값 안전 체크 (전송 전)
        double desired_joint = point.q[id];
        double diff = desired_joint - tmotor->current_joint_angle;
        if (std::abs(diff) > POS_DIFF_LIMIT) {
            std::cerr << "[Controller] TMotor 급변 차단 (" << tmotor->name << ")"
                      << "  diff=" << diff * 180.0 / M_PI << "deg\n";
            continue;
        }
        if (desired_joint < tmotor->min_angle || desired_joint > tmotor->max_angle) {
            std::cerr << "[Controller] TMotor 범위 초과 차단 (" << tmotor->name << ")"
                      << "  target=" << desired_joint * 180.0 / M_PI << "deg\n";
            continue;
        }
 
        if (mode == ControlMode::POS) {
            t_codec.setPosition(tmotor->node_id, &frame, static_cast<float>(motor_position));

            std::vector<double> values = {(double)tmotor->id,
                1.0,    // mode
                motor_position,
                tmotor->current_position,
                motor_position - tmotor->current_position,
                tmotor->current_current
            };
            motor_log.record(values);
        } else if (mode == ControlMode::VEL) {
            double err = motor_position - tmotor->current_position;
            double ermp = motor_velocity * tmotor->pole * tmotor->gear_ratio * 60.0 / 2.0 / M_PI;

            double control_input = ermp + tmotor->control_gain * err;
            control_input = std::clamp(control_input, -100000.0, 100000.0);

            t_codec.setVelocity(tmotor->node_id, &frame, static_cast<float>(control_input));

            std::vector<double> values = {(double)tmotor->id,
                2.0,    // mode
                motor_position,
                tmotor->current_position,
                motor_position - tmotor->current_position,
                tmotor->current_current,
                control_input
            };
            motor_log.record(values);
        } else if (mode == ControlMode::None) {
            std::cerr << "[Controller] TMotor ControlMode 미설정 (" << tmotor->name << ")\n";
            continue;
        }
 
        robot.can.sendFrame(tmotor->socket, frame);
    }
}

void Controller::maxon_motor_send_task(const ControlSetPoint &point) {
    struct can_frame frame;
 
    for (auto &[id, motor] : robot.motors) {
        auto maxon = std::dynamic_pointer_cast<MaxonMotor>(motor);
        if (!maxon) continue;
 
        ControlMode mode = point.mode[id];
        double motor_position  = maxon->joint_angle_to_motor_position(point.q[id]);
 
        if (mode == ControlMode::CSP) {
            m_codec.setPosition(maxon->tx_pdo_ids[1], &frame, motor_position);

            std::vector<double> values = {(double)maxon->id,
                1.0,    // mode
                motor_position,
                maxon->current_position,
                motor_position - maxon->current_position,
                maxon->current_torque
            };
            motor_log.record(values);
        } else if (mode == ControlMode::CST) {
            double torque_mNm = cal_torque(maxon, motor_position);
            m_codec.setTorque(maxon->tx_pdo_ids[3], &frame, static_cast<int>(torque_mNm));

            std::vector<double> values = {(double)maxon->id,
                0.0,    // mode
                motor_position,
                maxon->current_position,
                motor_position - maxon->current_position,
                maxon->current_torque,
                torque_mNm
            };
            motor_log.record(values);
        } else if (mode == ControlMode::CSV) {
            std::cerr << "[Controller] MaxonMotor CSV 모드 구현 안됨 (" << maxon->name << ")\n";
            continue;
        } else if (mode == ControlMode::None) {
            std::cerr << "[Controller] MaxonMotor ControlMode 미설정 (" << maxon->name << ")\n";
            continue;
        }
 
        robot.can.sendFrame(maxon->socket, frame);
    }
}

double Controller::cal_torque(std::shared_ptr<MaxonMotor> &maxon, double target_position) {
    const double dt = 0.001;    // 1ms
    const double alpha = 0.2;   // 저역통과 필터 계수
 
    double err = target_position - maxon->current_position;
    double err_dot_raw = (err - maxon->prev_err) / dt;

    double err_dot_filtered = alpha * err_dot_raw + (1.0 - alpha) * maxon->prev_err_dot; // 근사 필터
    maxon->prev_err = err;
    maxon->prev_err_dot = err_dot_filtered;

    double torque_mNm = maxon->control_kp * err + maxon->control_kd * err_dot_filtered;

    // 중력 보상
    double gravity_angle = 0.0;
    for (auto &[other_id, motor] : robot.motors) {
        auto tmotor = std::dynamic_pointer_cast<TMotor>(motor);
        if (!tmotor) continue;

        bool is_right_wrist = (maxon->name == "right_wrist") &&
                              (tmotor->name == "right_shoulder_2" || tmotor->name == "right_elbow");
        bool is_left_wrist  = (maxon->name == "left_wrist") &&
                              (tmotor->name == "left_shoulder_2"  || tmotor->name == "left_elbow");

        if (is_right_wrist || is_left_wrist) {
            gravity_angle += tmotor->current_joint_angle;
        }
    }
    gravity_angle += maxon->current_joint_angle;

    double gravity_torque_Nm = STICK_MASS_KG * 9.81 * STICK_LEN_M * std::sin(gravity_angle) / maxon->gear_ratio;
    torque_mNm -= gravity_torque_Nm * 1000.0;  // N·m -> mN·m

    return torque_mNm;
}

void Controller::dynamicxel_send_task(const ControlSetPoint &point) {
    for (auto &[id, motor] : robot.motors) {
        auto dxl = std::dynamic_pointer_cast<DynamixelMotor>(motor);
        if (!dxl) continue;
 
        double motor_position  = dxl->joint_angle_to_motor_position(point.q[id]);
 
        std::vector<double> dxl_command = {0.0, 0.0, motor_position};   // 속도, 가속도가 0인 경우 계단 입력
        dxl->write_command(dxl_command);

        std::pair<bool, double> data = dxl->read_data();                // dxl은 별개로 read (5ms 주기)
        if (data.first) {
            dxl->current_position = data.second;
            dxl->current_joint_angle = dxl->motor_position_to_joint_angle(data.second);
        }

        std::vector<double> values = {(double)dxl->id,
            motor_position,
            dxl->current_position,
            motor_position - dxl->current_position
        };
        motor_log.record(values);
    }
}

// ===== RECV =====
void Controller::read_frames() {
    struct can_frame frame;

    for (auto &[ifname, socket_fd] : robot.can.getSocket()) {
        while (true) {
            ssize_t bytes = read(socket_fd, &frame, sizeof(frame));
            if (bytes == sizeof(frame)) {
                temp_frames[socket_fd].push_back(frame);
            } else if (bytes < 0 && (errno == EWOULDBLOCK || errno == EAGAIN)) {
                break;  // 더 읽을 프레임 없음
            } else {
                break;  // 기타 오류
            }
        }
    }
}

void Controller::distribute_frames() {
    bool is_safe = true;

    for (auto &[id, motor_ptr] : robot.motors) {
        if (auto tmotor = std::dynamic_pointer_cast<TMotor>(motor_ptr)) {
            for (auto &frame : temp_frames[tmotor->socket]) {
                if ((frame.can_id & 0xFF) == tmotor->node_id) {
                    auto [mid, pos, spd, cur, temp, err] = t_codec.parseRecieveCommand(&frame);
                    tmotor->current_position     = pos;
                    tmotor->current_velocity     = spd;
                    tmotor->current_current      = cur;
                    tmotor->current_joint_angle  = tmotor->motor_position_to_joint_angle(pos);

                    if (!safety_check_recv_tmotor(tmotor)) {
                        is_safe = false;
                    }
                }
            }
        } else if (auto maxon = std::dynamic_pointer_cast<MaxonMotor>(motor_ptr)) {
            for (auto &frame : temp_frames[maxon->socket]) {
                if (frame.can_id == maxon->rx_pdo_ids[0]) {
                    auto [mid, pos, torque, status] = m_codec.parseRecieveCommand(&frame);
                    maxon->current_position    = pos;
                    maxon->current_torque      = torque;
                    maxon->status_bit          = status;
                    maxon->current_joint_angle = maxon->motor_position_to_joint_angle(pos);

                    if (!safety_check_recv_maxon(maxon)) {
                        is_safe = false;
                    }
                }
            }
        }
    }

    temp_frames.clear();

    if (!is_safe) {
        ctx.running = false;
    }
}

bool Controller::safety_check_recv_tmotor(std::shared_ptr<TMotor> &motor) {
    double angle = motor->current_joint_angle;

    if (angle < motor->min_angle || angle > motor->max_angle) {
        std::cerr << "[Controller] TMotor 범위 초과 (" << motor->name << ")"
                  << "  joint=" << angle * 180.0 / M_PI << "deg\n";
        return false;
    }

    if (motor->current_current > motor->current_limit) {
        if (motor->cnt++ > 5) {
            std::cerr << "[Controller] TMotor 전류 초과 (" << motor->name << ")"
                    << "  current=" << motor->current_current << "A\n";
            return false;
        }
    } else {
        motor->cnt = 0;
    }

    return true;
}

bool Controller::safety_check_recv_maxon(std::shared_ptr<MaxonMotor> &motor) {
    double angle = motor->current_joint_angle;

    if (angle < motor->min_angle || angle > motor->max_angle) {
        std::cerr << "[Controller] MaxonMotor 범위 초과 (" << motor->name << ")"
                  << "  joint=" << angle * 180.0 / M_PI << "deg\n";

        // getShutdown 송신
        struct can_frame frame;
        m_codec.getShutdown(motor->tx_pdo_ids[0], &frame);
        robot.can.sendFrame(motor->socket, frame);

        struct can_frame sync_frame;
        m_codec.getSync(&sync_frame);
        robot.can.sendFrame(motor->socket, sync_frame);

        return false;
    }
    return true;
}
