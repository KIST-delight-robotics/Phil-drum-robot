# Phil

> 개발 중인 드럼 연주 로봇 제어 시스템.

CAN 통신 기반으로 TMotor / Maxon / Dynamixel 모터를 제어하며, 키보드 입력 또는 LLM(TCP)을 통해 명령을 수신합니다. 이름은 드러머 Phil Collins에서 따왔습니다.

---

## 시스템 요구사항

### 운영체제 및 권한

- **OS**: Ubuntu 22.04 LTS
- **권한**: `SCHED_FIFO` 우선순위 설정을 위해 `sudo` 실행 필요

### 빌드 환경

- **컴파일러**: g++ (C++17 지원)
- **빌드 도구**: GNU Make
- **최적화 옵션**: `-O2 -g` (Makefile에 기본 설정)

### 의존성 (저장소에 포함)

- **Dynamixel SDK** — `drumrobot/lib/dynamixel_sdk/`
- **nlohmann/json** — `drumrobot/lib/nlohmann/json.hpp`

### 하드웨어

- USB-CAN 어댑터 (`can0`, `can1` 등)
- Dynamixel U2D2 (`/dev/ttyUSB0`)

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
   TrajectoryGenerator (궤적 생성 → ControlSetPoint 시퀀스)
        ↓
   ControlQueue        (5ms 주기 ControlSetPoint)
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
| `BehaviorPlanner` | `ParsedCommand`를 받아 `MotionPrimitive` 시퀀스 생성 |
| `TrajectoryGenerator` | `MotionPrimitive`를 받아 5ms 단위 `ControlSetPoint` 궤적 생성 |
| `MotionPlanner` | 위 세 컴포넌트를 orchestrate하는 스레드 루프. `CommandQueue` 소비 및 `ControlQueue` 잔량 관리 |
| `KinematicsSolver` | 손 끝 좌표 → 9개 관절각 (역기구학), 9개 관절각 → 손 끝 좌표 (순기구학) |
| `Controller` | `ControlQueue`에서 꺼내 CAN 프레임 송신 (1ms Maxon 보간 + 5ms TMotor 동시), 모터 상태 수신 및 안전 검사 |
| `Robot` | 모터 객체들의 컨테이너. 초기화, socket 할당, Maxon 설정 |
| `CanInterface` | CAN 포트 활성화, 소켓 생성, 프레임 read/write, Non-block 설정 |
| `MotorCodec` | TMotor / MaxonMotor 종류별 CAN 프레임 인코딩·디코딩 |
| `Logger` | 모터 상태를 타임스탬프 기반 CSV 파일로 저장 |

### 스레드 구조

| 스레드 | 우선순위 | 주기 | 역할 |
|---|---|---|---|
| `send_thread` | 40 | 1ms (Maxon 보간) / 5ms (TMotor + Maxon + Dynamixel) | CAN 프레임 송신 |
| `recv_thread` | 30 | 100µs | CAN 프레임 수신 및 모터 상태 갱신, 안전 검사 |
| `motion_planning_thread` | 20 | 5ms 폴링 | CommandQueue → MotionQueue → ControlQueue 변환 |
| `keyboard_input_thread` / `tcp_server_thread` | 10 | 블로킹 | 사용자 입력 수신 |

우선순위는 `SCHED_FIFO` 정책 기준.

---

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
    │   ├── motors.json                     # 모터 설정 (ID, 관절 범위, PDO ID 등)
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
    │   │   ├── app_context.hpp             # 전역 상태 플래그 (running, send/recv_active, shutdown_requested)
    │   │   ├── command_queue.hpp           # 문자열 명령 큐 (mutex 보호)
    │   │   ├── control_queue.hpp           # ControlSetPoint 큐 (mutex 보호)
    │   │   └── motion_queue.hpp            # MotionPrimitive 큐 (mutex 보호)
    │   ├── hardware/
    │   │   ├── can_interface.hpp           # CAN 소켓 관리
    │   │   ├── motor.hpp                   # Motor / TMotor / MaxonMotor / DynamixelMotor
    │   │   ├── motor_codec.hpp             # CAN 프레임 인코딩·디코딩 (TMotorServoCodec / TMotorMITCodec / MaxonMotorCodec)
    │   │   └── robot.hpp                   # 모터 집합 및 초기화
    │   ├── input/
    │   │   └── keyboard_handler.hpp        # 키보드 입력 → CommandQueue
    │   ├── kinematics/
    │   │   └── kinematics_solver.hpp       # 기구학 (IK, FK) 솔버
    │   ├── net/
    │   │   └── tcp_server.hpp              # TCP 서버 → CommandQueue
    │   ├── realtime/
    │   │   └── controller.hpp              # CAN 송수신 루프, 안전 검사
    │   ├── trajectory/
    │   │   ├── behavior_planner.hpp        # ParsedCommand → MotionPrimitive 시퀀스
    │   │   ├── command_parser.hpp          # 문자열 → ParsedCommand (Opcode + args)
    │   │   ├── motion_planner.hpp          # 플래닝 스레드 orchestrator
    │   │   └── trajectory_generator.hpp    # MotionPrimitive → ControlSetPoint 궤적 (관절/작업공간)
    │   └── util/
    │       └── logger.hpp                  # CSV 로깅 유틸리티
    └── src/
        ├── main.cpp                        # 스레드 생성 및 우선순위 설정
        ├── common/ ...
        ├── hardware/ ...
        ├── input/ ...
        ├── kinematics/ ...
        ├── net/ ...
        ├── realtime/ ...
        ├── trajectory/ ...
        └── util/ ...
```

---

## 좌표계 / 운동학 컨벤션

### 단위

| 위치 | 단위 |
|---|---|
| 사용자 입력 (TCP / 키보드) | 도 (degree) |
| JSON 설정 파일 (`motors.json`, `robot_poses.json`, `kinematics.json`) | 도 (degree) |
| 코드 내부 (`q`, `qd`, 모든 관절각) | 라디안 (rad) |
| 모터 위치 (TMotor) | rad (모터 좌표계, `direction_sign` 및 `initial_joint_angle`로 변환) |
| 모터 위치 (Maxon) | rad (감속비 적용 전) |
| 작업공간 좌표 (`p_target_R`, `p_target_L`) | 미터 (m) |

### 좌표계

- 베이스 원점: 허리(waist) 회전축
- 어깨 간격: `link_length.waist` (0.520 m)
- 상완 길이: `link_length.upper_arm` (0.230 m)
- 하완 길이: `link_length.forearm` (0.200 m)
- 스틱 길이: `link_length.stick` (0.373 m)

값은 `kinematics.json` 참조.

### DH 컨벤션

- Modified DH (Craig) 사용
- 변환 행렬: `KinematicsSolver::dh_transform(a, alpha, d, theta)`
- 오른팔 / 왼팔 각각 6단 DH로 손 끝 좌표 계산

### 관절 방향

- 각 관절의 회전 방향은 `motors.json`의 `direction_sign` (±1.0)로 정의
- 모터 좌표 → 관절 좌표: `joint = motor * direction_sign + initial_joint_angle`

---

## 모터 구성

총 13개 관절. `motors.json`에 ID 순서대로 정의됨.

| ID | 이름 | 타입 | 모델 |
|---|---|---|---|
| 0  | waist            | TMotor     | AK10-9       |
| 1  | right_shoulder_1 | TMotor     | AK70-10      |
| 2  | left_shoulder_1  | TMotor     | AK70-10      |
| 3  | right_shoulder_2 | TMotor     | AK70-10      |
| 4  | right_elbow      | TMotor     | AK70-10      |
| 5  | left_shoulder_2  | TMotor     | AK70-10      |
| 6  | left_elbow       | TMotor     | AK70-10      |
| 7  | right_wrist      | MaxonMotor | DCX22L       |
| 8  | left_wrist       | MaxonMotor | DCX22L       |
| 9  | right_pedal      | MaxonMotor | DCX32L       |
| 10 | left_pedal       | MaxonMotor | DCX32L       |
| 11 | head_yaw         | Dynamixel  | XM430-W210-T |
| 12 | head_pitch       | Dynamixel  | XM430-W210-T |

### 지원 제어 모드

| 모터 | 제어 모드 |
|---|---|
| TMotor       | `POS` (SET_POS), `VEL` (SET_RPM + P 피드백), `SET_POS_SPD`, `SET_ORIGIN`, `CURRENT_BRAKE` |
| TMotor (MIT) | Position-Velocity-Torque 통합 제어 코덱 구현됨. 단, Controller에서 현재 미사용 |
| MaxonMotor   | `CSP` (Cyclic Sync Position), `CST` (Cyclic Sync Torque), `HMM` (Homing). `CSV` 미구현 |
| Dynamixel    | 위치 제어 (Profile Acceleration / Velocity + Goal Position) |

### 모터 통신

| 모터 | 물리 계층 | 프로토콜 |
|---|---|---|
| TMotor      | CAN 1Mbps                    | TMotor 독자 Servo / MIT 모드 |
| MaxonMotor  | CAN 1Mbps                    | CANopen (SDO / PDO) |
| Dynamixel   | UART 4.5Mbps (`/dev/ttyUSB0`) | Dynamixel Protocol 2.0 |

---

## 명령 프로토콜

### 패킷 형식

```
OPCODE|arg1|arg2\n
```

필드 구분자 `|`, 패킷 구분자 `\n`. TCP 및 키보드 모드 공통.

### Opcode 목록

| Opcode | 인자 | 설명 | 상태 |
|---|---|---|---|
| `START`     | 없음                                            | home 포즈로 이동 후 제어 활성화 | ✅ |
| `QUIT` / `Q`| 없음                                            | shutdown 포즈 이동 후 시스템 종료 | ✅ |
| `LOOK`      | `pan_deg`, `tilt_deg`                          | 머리 yaw / pitch 제어 | ✅ |
| `GESTURE`   | `type`                                          | 제스처 실행 (`nod` / `shake` / `wave` / `hi` / `hurray` / `happy`) | ✅ |
| `MOVE`      | `motor_name`, `angle_deg`, `[move_time=2.0]`   | 개별 관절 이동 | ✅ |
| `POSE`      | `pose_name`                                     | 사전 정의 포즈로 이동 (`home` / `ready` / `shutdown`) | ✅ |
| `HIT`       | `target`                                        | 드럼 타격 | ⚠️ 미구현 (프레임워크만 존재) |

### 명령 예시

```
START                       # 시스템 시작 (home 포즈로 이동)
POSE|home                   # home 포즈로 이동
POSE|ready                  # ready 포즈로 이동
LOOK|0|10                   # 정면, 위쪽 10도 응시
LOOK|-30|0                  # 왼쪽 30도 응시
GESTURE|nod                 # 끄덕임
GESTURE|wave                # 손 흔들기
GESTURE|hurray              # 환호
MOVE|right_wrist|45         # right_wrist를 45도로 이동 (기본 2.0초)
MOVE|right_wrist|45|1.0     # right_wrist를 45도로 1.0초에 이동
MOVE|waist|-30|2.0          # 허리를 -30도로 2초에 이동
QUIT                        # shutdown 포즈 이동 후 종료
```

각도 단위는 모두 **도(degree)**이며 내부적으로 라디안으로 변환됩니다.

### 사전 정의 포즈 (`robot_poses.json`)

| 포즈 이름 | 설명 |
|---|---|
| `init`     | `motors.json`의 `initial_joint_angle`과 일치하는 초기 자세 |
| `home`     | 연주 대기 자세 |
| `ready`    | 준비 자세 |
| `shutdown` | 종료 전 안전 자세 |

---

## 궤적 프로파일

`TrajectoryGenerator`가 지원하는 보간 방식. `BehaviorPlanner`의 기본값은 `COSINE`.

| 프로파일 | 설명 |
|---|---|
| `COSINE`       | 사인 보간. 양 끝에서 속도 0, 부드러운 가속 / 감속 (기본값) |
| `CUBIC`        | 3차 다항식. 양 끝에서 속도 0 |
| `QUINTIC`      | 5차 다항식. 양 끝에서 속도·가속도 모두 0 |
| `TRAPEZOIDAL`  | 사다리꼴 속도 프로파일. 가속(25%) → 등속(50%) → 감속(25%) |

---

## 상태 플래그 (AppContext)

| 플래그 | 의미 |
|---|---|
| `running`            | 전체 종료 플래그. false가 되면 모든 스레드 루프 탈출 |
| `send_active`        | `Controller::send_loop` 활성화 신호. `MotionPlanner`가 첫 궤적을 `ControlQueue`에 적재하는 시점에 `recv_active`와 동시에 켜짐 |
| `recv_active`        | `Controller::recv_loop` 활성화 신호. `send_active`와 동일한 시점에 켜짐 |
| `shutdown_requested` | 새 명령 안 받겠다는 신호. ControlQueue 소진 후 `running = false` |

---

## 안전 메커니즘

### TMotor 전류 초과 차단

- 수신된 `current_current`가 `current_limit`을 **연속 5회 초과**하면 시스템 정지
- 일시적 과전류(스파이크)는 카운터가 리셋되어 무시됨
- 카운터 변수: `Motor::cnt`

### IK 실패 처리

- `KinematicsSolver::ik_solve()`가 `success = false`를 반환하면:
  - 해당 `MotionPrimitive` 폐기
  - 시스템은 **계속 동작** (다음 명령 처리 가능)
- 실패 원인: 도달 불가능한 좌표, NaN / Inf, 관절 한계 초과

### 송신 전 안전 검사 (TMotor만)

- **급변 차단**: 목표 위치와 현재 위치의 차이가 **10도(약 0.175 rad)** 이상이면 해당 모터 송신만 건너뜀
- **범위 검사**: `motors.json`의 `min_angle` / `max_angle` 범위를 벗어나면 송신 건너뜀
- MaxonMotor와 Dynamixel은 송신 전 안전 검사가 별도로 없음 (CSP 모드에서는 모터 자체 제한에 의존)

### 수신 후 안전 검사

- **TMotor 범위 초과** → 즉시 `ctx.running = false` (전체 시스템 정지)
- **Maxon 범위 초과** → `getShutdown` PDO 송신 후 전체 시스템 정지

복구 절차:

1. 시스템 종료 후 모터를 안전 범위로 수동 이동
2. CAN 포트 재기동 권장 (USB 전원 차단 후 재연결)
3. `motors.json` 한계 값 검토 후 재실행

---

## 빌드 및 실행

### 빌드

```bash
make
```

### 실행

키보드 모드:

```bash
sudo ./drumrobot/bin/main.out --mode keyboard
```

LLM 모드 (TCP 포트 1951):

```bash
sudo ./drumrobot/bin/main.out --mode llm
```

또는 최상위 Makefile 사용:

```bash
make run-keyboard
make run-llm
```

### TCP 클라이언트 (LLM 모드와 별도 터미널에서)

```bash
python3 planner/main.py
```

호스트 `127.0.0.1`, 포트 `1951`로 연결됩니다.

---

## 라이선스 및 외부 의존성

> ⚠️ 현재 저장소에 LICENSE 파일이 없습니다. 공개 전 라이선스 명시를 권장합니다.

### 포함된 외부 라이브러리

| 라이브러리 | 위치 | 라이선스 |
|---|---|---|
| nlohmann/json   | `drumrobot/lib/nlohmann/json.hpp` | MIT        |
| Dynamixel SDK   | `drumrobot/lib/dynamixel_sdk/`    | Apache 2.0 |

---

## 참고 사항

- **CAN bitrate**: 1Mbps (`can_interface.cpp::activateCanPort`)
- **Maxon 보간**: `send_loop`에서 5ms 구간을 1ms × 5 스텝으로 분할하여 CSP 위치 명령을 선형 보간 전송. 5ms 시점에서 TMotor / Maxon / Dynamixel을 동시 송신
- **`virtual_maxon_motor`**: 소켓당 Maxon 모터 1개를 대표로 선정해 Sync 프레임(0x80) 전송에 사용
- **Logger**: 런타임 시 `drumrobot/log/` 디렉토리에 `log_MMDD_HHmm_{name}.csv` 형식의 파일 생성. 헤더 컬럼: `t, id, mode, desired, actual, err, current/torque, input` (TMotor VEL / Maxon CST 기준. Maxon CSP·Dynamixel은 `input` 없이 기록)
- **모드 초기값**: TMotor = `VEL`, Wrist (Maxon) = `CST`, Pedal (Maxon) = `CSP` (`trajectory_generator.hpp` 참조)