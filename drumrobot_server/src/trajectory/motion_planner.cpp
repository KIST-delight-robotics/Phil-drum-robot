#include "trajectory/motion_planner.hpp"

#include "util/score_row.hpp"

MotionPlanner::MotionPlanner(AppContext &ctxRef, CommandQueue &commandQueueRef, ControlQueue &controlQueueRef, MotionQueue &motionQueueRef, Robot &robotRef, AudioPlayer &audioRef)
    : ctx(ctxRef), command_queue(commandQueueRef), control_queue(controlQueueRef), motion_queue(motionQueueRef), robot(robotRef),
    behavior_planner(ctxRef, robotRef, audioRef), trajectory_generator(ctxRef, control_queue), motion_log("motion_command") {}

MotionPlanner::~MotionPlanner() {}

void MotionPlanner::run() {
    initialize();

    while (ctx.running.load()) {
        if (auto cmd = command_queue.try_pop()) {
            // 명령이 있으면 motion_queue에 적재
            plan_motions(*cmd);
        }

        if (ctx.switch_pending) {
            do_switch();    // 이어치기 전환 (성공 시 motion_queue를 새 스트림으로 교체)
        }

        // control_queue 잔량이 임계값 이하면 다음 모션 생성
        if (control_queue.size() < threshold) {
            // send_active 이 후 motion_queue가 없으면 모션 채우기
            if (ctx.send_active.load() && motion_queue.empty()) {
                schedule_idle_motion();
            }

            if (auto motion = motion_queue.try_pop()) {
                trajectory_generator.generate_trajectory(*motion);
                track_parked_notes(*motion);

                if (!ctx.recv_active.load()) ctx.recv_active = true;
                if (!ctx.send_active.load()) ctx.send_active = true;
                if (ctx.play_abort.load()) abort_play_motion();

                record_motion(*motion);
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    ctx.running = false;
    std::cerr << "[MotionPlanner] 스레드 종료\n";
}

void MotionPlanner::initialize() {
    behavior_planner.init_poses_from_json();

    // init 포즈 vs 모터 initial_joint_angle 비교
    const auto &init_pose = behavior_planner.poses["init"];
    for (auto &[id, motor] : robot.motors) {
        auto dxl = std::dynamic_pointer_cast<DynamixelMotor>(motor);
        if (dxl) continue;  // 다이나믹셀은 초기 위치 비교 안함

        double diff = std::abs(init_pose[id] - motor->initial_joint_angle);

        if (diff > 1e-4) {
            std::cerr << "[MotionPlanner] init pose mismatch! "
                      << "joint " << id << " (" << motor->name << "): "
                      << "robot_poses.json="  << init_pose[id] * 180.0 / M_PI << "deg  "
                      << "motors.json="       << motor->initial_joint_angle * 180.0 / M_PI << "deg\n";
        }
    }

    trajectory_generator.initialize(behavior_planner.poses);
}

void MotionPlanner::plan_motions(const ParsedCommand& cmd) {
    if (ctx.robot_state.load() == RobotState::SHUTTINGDOWN) return; // 종료 상태가 되면 추가 명령 안받음

    std::vector<MotionPrimitive> motion_sequence = behavior_planner.generate_motion_sequence(cmd);

    int n = motion_sequence.size();
    if (n >= 1) motion_done = false;

    for (int i = 0; i < n; i++) {
        motion_queue.push(motion_sequence[i]);
    }

    record_command(cmd);
}

void MotionPlanner::schedule_idle_motion() {
    if (ctx.robot_state.load() == RobotState::SHUTTINGDOWN) {
        return; // 종료 상태가 되면 추가 명령 안받음
    } else if (ctx.robot_state.load() == RobotState::INIT) {
        // 키 제거하기 전 현재 위치 유지 (고정)
        MotionPrimitive standby_motion;

        standby_motion.type = MotionType::STANDBY;
        motion_queue.push(standby_motion);
    } else if (ctx.robot_state.load() == RobotState::IDLE) {
        // 동작 종료
        if (!motion_done) {
            motion_done = true;
            std::cerr << "[MotionPlanner] 동작을 마쳤습니다.\n";
        }

        // 대기 동작
        MotionPrimitive idle_motion;

        idle_motion.type = MotionType::IDLE;    // TODO: IDLE은 TRANSLATE 반복으로 대체 가능
        motion_queue.push(idle_motion);
    } else if (ctx.robot_state.load() == RobotState::PLAYING) {
        // 즉흥 연주 중이면 다음 악보로 이어붙임 (START/END 없이 체이닝)
        std::vector<MotionPrimitive> improv_chunk = behavior_planner.make_improv_chunk();
        if (!improv_chunk.empty()) {
            int chunk_size = static_cast<int>(improv_chunk.size());
            for (int i = 0; i < chunk_size; i++) {
                motion_queue.push(improv_chunk[i]);
            }
            return;     // 연주 계속 (play_id/state 유지)
        }

        // 연주 종료
        std::cerr << "[MotionPlanner] 연주를 마쳤습니다.\n";
        motion_done = true;

        {
            std::lock_guard<std::mutex> lock(ctx.play_mutex);
            ctx.play_id.clear();    // 연주 끝 -> 현재 곡 없음 (pause_point는 유지)
        }

        // 대기 동작
        ctx.robot_state = RobotState::IDLE;
        MotionPrimitive idle_motion;

        idle_motion.type = MotionType::IDLE;    // TODO: IDLE은 TRANSLATE 반복으로 대체 가능
        motion_queue.push(idle_motion);
    }
}

// 이어치기 전환: front window 절단 -> 커밋 구간 + 새 악보 window로 큐 교체.
// 실패하면 아무것도 바꾸지 않고 기존 연주를 계속한다.
void MotionPlanner::do_switch() {
    std::vector<std::string> switch_args = ctx.switch_args;
    ctx.switch_pending = false;
    ctx.switch_args.clear();

    if (ctx.robot_state.load() != RobotState::PLAYING) {
        std::cerr << "[MotionPlanner] 전환 불가: PLAYING 상태가 아닙니다\n";
        return;
    }
    if (ctx.play_abort.load()) {
        std::cerr << "[MotionPlanner] 전환 불가: 중단 처리 중입니다\n";
        return;
    }

    std::optional<MotionPrimitive> front_motion = motion_queue.try_peek();
    if (!front_motion.has_value() || front_motion.value().type != MotionType::DRUM ||
        front_motion.value().flag != PlayFlag::PLAYING || front_motion.value().robotic_drum_score.size() < 2) {
        std::cerr << "[MotionPlanner] 전환 불가: 절단할 연주 window가 없습니다 (잠시 후 재시도 가능)\n";
        return;
    }

    std::vector<MotionPrimitive> new_windows = behavior_planner.make_switch_windows(
        front_motion.value().robotic_drum_score, switch_args, parked_note_r, parked_note_l);
    if (new_windows.empty()) {
        std::cerr << "[MotionPlanner] 전환 실패: 기존 연주를 계속합니다\n";
        return;
    }

    motion_queue.clear();
    int window_count = static_cast<int>(new_windows.size());
    for (int i = 0; i < window_count; i++) {
        motion_queue.push(new_windows[i]);
    }
    std::cerr << "[MotionPlanner] 이어치기 전환 완료 (window " << window_count << "개)\n";
}

// 절단 시점의 손 위치 근거: 각 DRUM window의 rds[0]은 방금 확정된 타격이다.
void MotionPlanner::track_parked_notes(const MotionPrimitive& motion) {
    if (motion.type != MotionType::DRUM) {
        return;
    }
    if (motion.flag == PlayFlag::START) {
        parked_note_r = motion.init_note_r;
        parked_note_l = motion.init_note_l;
        return;
    }
    if (motion.flag != PlayFlag::PLAYING || motion.robotic_drum_score.empty()) {
        return;
    }
    const DrumEvent& front_event = motion.robotic_drum_score[0];
    int coord_r = open_hihat_note(front_event.note_num_R, front_event.is_closed_hihat);
    int coord_l = open_hihat_note(front_event.note_num_L, front_event.is_closed_hihat);
    if (coord_r != 0) {
        parked_note_r = coord_r;
    }
    if (coord_l != 0) {
        parked_note_l = coord_l;
    }
}

void MotionPlanner::abort_play_motion() {
    bool pause_requested = ctx.pause_requested.load();

    if (pause_requested) {
        save_pause_point();   // PAUSE: 잔여 모션 폐기 전에 재개 지점 저장
    } else {
        // stop / 내부 에러: 재개 지점 폐기 (RESUME 거부)
        std::lock_guard<std::mutex> lock(ctx.play_mutex);
        ctx.pause_point.clear();
    }

    // 즉흥 연주 PAUSE면 재개 지점을 improv 기준으로 다시 저장.
    behavior_planner.on_play_abort(pause_requested);

    ctx.pause_requested = false;

    motion_queue.clear();     // 남은 PLAYING 전부 폐기
    MotionPrimitive end; end.type = MotionType::DRUM; end.flag = PlayFlag::END;
    motion_queue.push(end);   // END(ready 복귀)만 남김

    ctx.play_abort = false;
    std::cerr << "[MotionPlanner] 연주가 비정상 종료됩니다.\n";
}

// 재개 지점 저장. motion_queue.clear() 전에 불러야 한다.
// 큐 맨 앞 window에서 아직 안 친 첫 이벤트의 마디가 재개 지점이다.
void MotionPlanner::save_pause_point() {
    std::optional<MotionPrimitive> front_motion = motion_queue.try_peek();

    std::lock_guard<std::mutex> lock(ctx.play_mutex);
    ctx.pause_point.clear();

    if (!front_motion.has_value()) return;      // 큐가 비어 있음 (곡이 사실상 끝난 상태)
    if (ctx.play_id.empty()) return;            // 연주 중인 곡을 모름

    const MotionPrimitive &motion = front_motion.value();
    if (motion.type != MotionType::DRUM) return;
    if (motion.flag != PlayFlag::PLAYING) return;           // START/END는 재개 지점이 없음
    if (motion.robotic_drum_score.size() < 2) return;

    // [0]은 이미 친 이벤트, [1]이 아직 안 친 첫 이벤트.
    // improv면 on_play_abort()가 improv 기준으로 다시 저장한다.
    ctx.pause_point.save(ctx.play_id,
                         static_cast<int>(motion.robotic_drum_score[1].bar),
                         motion.robotic_drum_score[0].row);

    std::cerr << "[MotionPlanner] 재개 지점 저장: id=" << ctx.pause_point.play_id
              << ", bar=" << ctx.pause_point.bar << "\n";
}

// ===== log =====
void MotionPlanner::record_command(const ParsedCommand& cmd) {
    // Opcode -> 문자열
    auto opcode_to_string = [](Opcode op) -> std::string {
        switch (op) {
            case Opcode::LOOK:    return "LOOK";
            case Opcode::GESTURE: return "GESTURE";
            case Opcode::MOVE:    return "MOVE";
            case Opcode::POSE:    return "POSE";
            case Opcode::HIT:     return "HIT";
            case Opcode::PLAY:    return "PLAY";
            case Opcode::PLAY_CTRL: return "PLAY_CTRL";
            case Opcode::START:   return "START";
            case Opcode::READY:   return "READY";
            case Opcode::PAUSE:   return "PAUSE";
            case Opcode::RESUME:  return "RESUME";
            case Opcode::GET_STATUS: return "GET_STATUS";
            case Opcode::QUIT:    return "QUIT";
            case Opcode::UNKNOWN: return "UNKNOWN";
            default:              return "UNKNOWN";
        }
    };

    std::vector<std::string> log = {"CMD"};

    log.push_back(opcode_to_string(cmd.opcode));
    log.push_back(cmd.valid ? "valid" : "invalid");

    // 인자 (OPCODE|arg1|arg2... 형태 그대로)
    for (const auto& arg : cmd.args) {
        log.push_back(arg);
    }

    motion_log.record(log);
}

void MotionPlanner::record_motion(const MotionPrimitive& motion) {
    auto profile_to_string = [](TrajectoryProfile p) -> std::string {
        switch (p) {
            case TrajectoryProfile::TRAPEZOIDAL: return "trapezoidal";
            case TrajectoryProfile::CUBIC:       return "cubic";
            case TrajectoryProfile::QUINTIC:     return "quintic";
            case TrajectoryProfile::COSINE:      return "cosine";
            default:                             return "?";
        }
    };
    auto space_to_string = [](TrajectorySpace s) -> std::string {
        return s == TrajectorySpace::JOINT ? "joint" : "task";
    };
    auto flag_to_string = [](PlayFlag f) -> std::string {
        switch (f) {
            case PlayFlag::START:   return "start";
            case PlayFlag::PLAYING: return "playing";
            case PlayFlag::END:     return "end";
            default:                return "?";
        }
    };

    std::vector<std::string> log = {"MOTION"};

    switch (motion.type) {
        case MotionType::IDLE:
            log.push_back("idle");
            break;

        case MotionType::STANDBY:
            log.push_back("standby");
            break;

        case MotionType::TRANSLATE:
            log.push_back("translate");
            log.push_back(space_to_string(motion.space));
            log.push_back(profile_to_string(motion.profile));
            log.push_back("t_total=" + std::to_string(motion.t_total));
            if (motion.space == TrajectorySpace::JOINT) {
                log.push_back("q_target_n=" + std::to_string(motion.q_target.size()));
                for (double q : motion.q_target) {
                    log.push_back(std::to_string(q));
                }
            } else {
                // task space: 오른팔/왼팔 목표 좌표
                std::string pr = "R(";
                for (size_t i = 0; i < motion.p_target_R.size(); ++i) {
                    pr += std::to_string(motion.p_target_R[i]);
                    if (i + 1 < motion.p_target_R.size()) pr += ",";
                }
                pr += ")";
                std::string pl = "L(";
                for (size_t i = 0; i < motion.p_target_L.size(); ++i) {
                    pl += std::to_string(motion.p_target_L[i]);
                    if (i + 1 < motion.p_target_L.size()) pl += ",";
                }
                pl += ")";
                log.push_back(pr);
                log.push_back(pl);
            }
            break;

        case MotionType::DRUM:
            log.push_back("drum");
            log.push_back(flag_to_string(motion.flag));
            log.push_back("events=" + std::to_string(motion.robotic_drum_score.size()));
            // 구간의 시간 범위 (첫/마지막 이벤트 누적 시간)
            if (!motion.robotic_drum_score.empty()) {
                log.push_back("t_first=" + std::to_string(motion.robotic_drum_score.front().t));
                log.push_back("t_last="  + std::to_string(motion.robotic_drum_score.back().t));
                log.push_back("bar_first=" + std::to_string(motion.robotic_drum_score.front().bar));
                log.push_back("bar_last="  + std::to_string(motion.robotic_drum_score.back().bar));
            } else if (flag_to_string(motion.flag) == "start") {
                log.push_back("init_note_r="  + std::to_string(motion.init_note_r));
                log.push_back("init_note_l="  + std::to_string(motion.init_note_l));
            }
            break;
    }

    motion_log.record(log);
}
