# Phil

> 개발 중인 드럼 연주 로봇 제어 시스템.

CAN 통신 기반으로 TMotor / Maxon / Dynamixel 모터를 제어하며, 키보드 입력 또는 LLM(TCP)을 통해 명령을 수신합니다. 이름은 드러머 Phil Collins에서 따왔습니다.

---

## 시스템 구조

### 데이터 흐름

```
입력 (키보드 or TCP)
        ↓
   CommandQueue        (문자열 명령)
        ↓
   CommandParser       (명령 파싱 → ParsedCommand)
        ↓
   BehaviorPlanner     (ParsedCommand → MotionPrimitive 시퀀스)
        ↓
   MotionQueue         (MotionPrimitive)
        ↓
   TrajectoryGenerator (궤적 생성 → ControlData 시퀀스)
        ↓
   ControlQueue        (5ms 주기 ControlData)
        ↓
   Controller          (CAN 송수신, 1ms / 5ms 주기)
        ↓
   CAN Bus → 모터
```

### 핵심 컴포넌트

| 컴포넌트 | 책임 |
|---|---|
| `KeyboardHandler` / `TcpServer` | 사용자 입력을 받아 `CommandQueue`에 푸시 |
| `CommandParser` | 문자열 명령을 파싱해 `ParsedCommand`(Opcode + args)로 변환 |
| `BehaviorPlanner` | `ParsedCommand`를 받아 `MotionPrimitive` 시퀀스 생성 → `MotionQueue`에 푸시 |
| `TrajectoryGenerator` | `MotionPrimitive`를 받아 5ms 단위 `ControlData` 궤적 생성 → `ControlQueue`에 푸시 |
| `MotionPlanner` | 위 세 컴포넌트를 orchestrate하는 스레드 루프. `CommandQueue` 소비 및 `ControlQueue` 잔량 관리 |
| `KinematicsSolver` | 손 끝 좌표 → 9개 관절각 (역기구학) |
| `Controller` | `ControlQueue`에서 꺼내 CAN 프레임 송신, 모터 상태 수신 |
| `Robot` | 모터 객체들의 컨테이너, 초기화 및 socket 관리 |
| `CanInterface` | CAN 포트 활성화, 소켓 생성, 프레임 read/write |
| `MotorCodec` | 모터 종류별 CAN 프레임 인코딩/디코딩 |
| `Logger` | 모터 상태 로그를 타임스탬프 기반 CSV 파일로 저장 |

### 스레드 구조

| 스레드 | 우선순위 | 주기 | 역할 |
|---|---|---|---|
| `send_thread` | 40 | 1ms / 5ms | CAN 프레임 송신 (Maxon 보간 + TMotor 동시 송신) |
| `recv_thread` | 30 | 100µs | CAN 프레임 수신 및 상태 갱신 |
| `motion_planning_thread` | 20 | 5ms | CommandQueue → MotionQueue → ControlQueue 변환 |
| `keyboard_input_thread` 또는 `tcp_server_thread` | 10 | 블로킹 | 사용자 입력 수신 |

## 파일 구조

```
Phil/
├── Makefile
├── planner/
│   └── main.py                             # TCP 클라이언트 (LLM 모드용)
└── drumrobot/
    ├── Makefile
    ├── bin/
    │   └── main.out
    ├── obj/
    ├── log/                                # Logger 출력 디렉토리 (런타임 생성)
    ├── config/
    │   ├── motors.json                     # 모터 설정 (ID, 관절 범위, PDO 등)
    │   ├── robot_poses.json                # 사전 정의 포즈 (init, home, ready, shutdown)
    │   └── kinematics.json                 # 관절 한계, 링크 길이
    ├── lib/
    │   ├── dynamixel_sdk/                  # Dynamixel SDK (소스 포함)
    │   │   ├── include/
    │   │   └── src/
    │   └── nlohmann/
    │       └── json.hpp
    ├── include/
    │   ├── common/
    │   │   ├── app_context.hpp             # 전역 상태 플래그 (running, send/recv active)
    │   │   ├── command_queue.hpp           # 문자열 명령 큐 (thread-safe)
    │   │   ├── control_queue.hpp           # 제어값 큐 (thread-safe)
    │   │   └── motion_queue.hpp            # MotionPrimitive 큐 (thread-safe)
    │   ├── hardware/
    │   │   ├── can_interface.hpp           # CAN 소켓 관리
    │   │   ├── motor.hpp                   # Motor / TMotor / MaxonMotor / DynamixelMotor
    │   │   ├── motor_codec.hpp             # CAN 프레임 인코딩·디코딩
    │   │   └── robot.hpp                   # 모터 집합 및 초기화
    │   ├── input/
    │   │   └── keyboard_handler.hpp        # 키보드 입력 → CommandQueue
    │   ├── kinematics/
    │   │   └── kinematics_solver.hpp       # 기구학 / 역기구학
    │   ├── net/
    │   │   └── tcp_server.hpp              # TCP 서버 → CommandQueue
    │   ├── realtime/
    │   │   └── controller.hpp              # CAN 송수신 루프
    │   ├── trajectory/
    │   │   ├── behavior_planner.hpp        # ParsedCommand → MotionPrimitive 시퀀스
    │   │   ├── command_parser.hpp          # 문자열 → ParsedCommand (Opcode + args)
    │   │   ├── motion_planner.hpp          # 플래닝 스레드 orchestrator
    │   │   └── trajectory_generator.hpp    # MotionPrimitive → ControlData 궤적
    │   └── util/
    │       └── logger.hpp                  # CSV 로깅 유틸리티
    └── src/
        ├── main.cpp                        # 스레드 생성 및 우선순위 설정
        ├── common/
        │   ├── command_queue.cpp
        │   ├── control_queue.cpp
        │   └── motion_queue.cpp
        ├── hardware/
        │   ├── can_interface.cpp
        │   ├── motor.cpp
        │   ├── motor_codec.cpp
        │   └── robot.cpp
        ├── input/
        │   └── keyboard_handler.cpp
        ├── kinematics/
        │   └── kinematics_solver.cpp
        ├── net/
        │   └── tcp_server.cpp
        ├── realtime/
        │   └── controller.cpp
        ├── trajectory/
        │   ├── behavior_planner.cpp
        │   ├── command_parser.cpp
        │   ├── motion_planner.cpp
        │   └── trajectory_generator.cpp
        └── util/
            └── logger.cpp
```

## 모터 구성

총 13개 관절. ID 순서대로 motors.json에 정의.

| ID | 이름 | 타입 | 비고 |
|---|---|---|---|
| 0 | waist | TMotor | AK10_9 |
| 1 | right_shoulder_1 | TMotor | AK70_10 |
| 2 | left_shoulder_1 | TMotor | AK70_10 |
| 3 | right_shoulder_2 | TMotor | AK70_10 |
| 4 | right_elbow | TMotor | AK70_10 |
| 5 | left_shoulder_2 | TMotor | AK70_10 |
| 6 | left_elbow | TMotor | AK70_10 |
| 7 | right_wrist | MaxonMotor | DCX22L |
| 8 | left_wrist | MaxonMotor | DCX22L |
| 9 | right_pedal | MaxonMotor | DCX32L |
| 10 | left_pedal | MaxonMotor | DCX32L |
| 11 | head_yaw | DynamixelMotor | XM430-W210-T |
| 12 | head_pitch | DynamixelMotor | XM430-W210-T |

### 지원 제어 모드

| 모터 | 제어 모드 |
|---|---|
| TMotor (Servo) | SET_POS, SET_RPM, SET_POS_SPD, SET_ORIGIN, CURRENT_BRAKE |
| TMotor (MIT) | Position-Velocity-Torque 통합 제어 |
| MaxonMotor | CSP (Cyclic Sync Position), CSV (미구현), CST (Cyclic Sync Torque), HMM (Homing) |
| DynamixelMotor | 위치제어 모드 |

### 모터 통신

| 모터 | 물리 계층 | 프로토콜 | 프레임 포멧 |
|---|---|---|_--|
| TMotor | CAN | TMotor 독자 | Standard |
| MaxonMotor | CAN | CANopen | Extended |
| DynamixelMotor | UART | Dynamixel Protocol 2.0 |---|

---

## 명령 프로토콜

### 패킷 형식 (TCP / 키보드)

```
OPCODE | arg1 | arg2 \n
```

필드 구분자 `|`, 패킷 구분자 `\n`.

### Opcode 목록

| Opcode | 설명 | 구현 상태 |
|---|---|---|
| `MOVE` | 지정 포즈/위치로 이동 | 개발 중 |
| `TURN` | 허리 회전 | 개발 중 |
| `PICK` | 드럼 타격 | 개발 중 |
| `SPEAK` | 발화 (미사용 예정) | 개발 중 |

### MotionType

| 타입 | 설명 |
|---|---|
| `TRAPEZOIDAL` | 사다리꼴 속도 프로파일 (가속·등속·감속) |
| `QUINTIC` | 5차 다항식 궤적 (미구현) |
| `DRUM_HIT` | 드럼 타격 전용 궤적 (미구현) |

---

## 상태 플래그 (AppContext)

| 플래그 | 의미 |
|---|---|
| `running` | 전체 종료 플래그. false 되면 모든 스레드 루프 탈출 |
| `send_active` | send_loop 활성화 신호 |
| `recv_active` | recv_loop 활성화 신호 |
| `shutdown_requested` | 새 명령 안 받겠다는 신호. 큐 소진 후 종료 |

---

## 빌드 및 실행

```bash
# 키보드 모드 (기본)
make run-keyboard

# LLM 모드 (TCP 포트 1951에서 명령 대기)
make run-llm

# TCP 클라이언트 실행 (LLM 모드와 함께 별도 터미널에서)
python3 planner/main.py
```

`SCHED_FIFO` 우선순위 설정 때문에 `sudo` 권한 필요.

---

## 안전 메커니즘

### 송신 전 (Controller::tmotor_send_task)
- **급변 차단**: 목표 위치와 현재 위치 차이가 10도 이상이면 송신 차단
- **범위 검사**: 목표 위치가 motors.json의 min/max 벗어나면 송신 차단
- **모드 검사**: ControlMode가 None/CSV(maxon)/지원되지 않는 모드면 송신 차단 + 에러 로그

### 수신 후 (Controller::safety_check_recv_*)
- **TMotor**: 현재 관절각이 범위 벗어나면 `ctx.running = false`로 시스템 정지
- **MaxonMotor**: 범위 벗어나면 QuickStop 송신 후 정지

---

## 참고

- **MotorCodec**: TMotor는 Servo / MIT 두 가지 통신 모드 지원. MaxonMotor는 CANopen 기반으로 SDO/PDO 사용
- **CAN bitrate**: 1Mbps (`can_interface.cpp::activateCanPort`)
- **virtual_maxon_motor**: 소켓당 1개의 Maxon 모터를 대표로 골라서 Sync 프레임 송신용으로 사용
- **Logger**: 런타임 시 `drumrobot/log/` 디렉토리에 타임스탬프 기반 CSV 파일 생성