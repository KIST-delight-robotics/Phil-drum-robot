# Phil

> 현재 개발 중입니다.

## 개요

드럼 로봇 제어 시스템. CAN 통신 기반으로 TMotor / Maxon / Dynamixel 모터를 제어하며, 키보드 입력 또는 LLM(TCP)을 통해 명령을 수신합니다.

## 파일 구조

```
Phil/
├── Makefile
├── planner/
│   └── main.py                         # TCP 클라이언트 (LLM 모드용)
└── drumrobot/
    ├── Makefile
    ├── bin/
    │   └── main.out
    ├── obj/
    ├── config/
    │   └── motors.json                 # 모터 설정 (ID, 관절 범위, PDO 등)
    ├── lib/
    │   └── nlohmann/
    │       └── json.hpp
    ├── include/
    │   ├── common/
    │   │   ├── command_queue.hpp       # 문자열 명령 큐 (thread-safe)
    │   │   └── control_queue.hpp       # 제어값 큐 (thread-safe)
    │   ├── hardware/
    │   │   ├── can_interface.hpp       # CAN 소켓 관리
    │   │   ├── motor.hpp               # Motor / TMotor / MaxonMotor / Dynamixel
    │   │   ├── motor_codec.hpp         # CAN 프레임 인코딩·디코딩
    │   │   └── robot.hpp               # 모터 집합 및 초기화
    │   ├── input/
    │   │   └── keyboard_handler.hpp    # 키보드 입력 → CommandQueue
    │   ├── net/
    │   │   └── tcp_server.hpp          # TCP 서버 → CommandQueue
    │   ├── realtime/
    │   │   └── controller.hpp          # CAN 송수신 루프 (1ms / 5ms)
    │   └── trajectory/
    │       └── motion_planner.hpp      # 명령 파싱 → ControlQueue
    └── src/
        ├── main.cpp                    # 스레드 생성 및 우선순위 설정
        ├── common/
        │   ├── command_queue.cpp
        │   └── control_queue.cpp
        ├── hardware/
        │   ├── can_interface.cpp
        │   ├── motor.cpp
        │   ├── motor_codec.cpp
        │   └── robot.cpp
        ├── input/
        │   └── keyboard_handler.cpp
        ├── net/
        │   └── tcp_server.cpp
        ├── realtime/
        │   └── controller.cpp
        └── trajectory/
            └── motion_planner.cpp
```

## 스레드 구조

| 스레드 | 우선순위 | 역할 |
|---|---|---|
| `send_thread` | 40 | 1ms / 5ms 주기로 CAN 프레임 송신 |
| `recv_thread` | 30 | 100µs 주기로 CAN 프레임 수신 |
| `motion_planning_thread` | 20 | CommandQueue → ControlQueue 변환 |
| `keyboard_input_thread` or `tcp_server_thread` | 10 | 명령 입력 수신 |

## 실행

```bash
# 키보드 모드 (기본)
make run-keyboard

# LLM 모드 (TCP 포트 1951)
make run-llm

# TCP 클라이언트 실행 (LLM 모드와 함께)
python3 planner/main.py
```

## 지원 모터

| 타입 | 통신 모드 | 비고 |
|---|---|---|
| TMotor | CAN (Servo / MIT) | waist, shoulder, elbow |
| MaxonMotor | CAN (CSP / CSV / CST / HMM) | wrist, pedal |
| Dynamixel | - | head (미구현) |
