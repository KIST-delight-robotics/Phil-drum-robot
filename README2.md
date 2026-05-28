# README 추가 제안 내용

> 기존 README에 추가하면 좋을 섹션들을 정리한 문서입니다.
> 각 섹션은 기존 README의 적절한 위치(빌드/실행 섹션 전후, 참고 사항 섹션 부근)에 삽입하시면 됩니다.

---

## 시스템 요구사항

### 운영체제 및 커널
- **OS**: Ubuntu 20.04 LTS 이상 (22.04 권장)
- **커널**: 실시간성 향상을 위해 `PREEMPT_RT` 패치 커널 권장 (필수 아님)
- **권한**: `SCHED_FIFO` 우선순위 설정을 위해 `sudo` 실행 필요

### 빌드 환경
- **컴파일러**: g++ (C++17 지원)
- **빌드 도구**: GNU Make
- **최적화 옵션**: `-O2 -g` (Makefile에 기본 설정)

### 필요 패키지
```bash
sudo apt update
sudo apt install build-essential iproute2 can-utils
```

### 의존성 (저장소에 포함)
- **Dynamixel SDK** (`drumrobot/lib/dynamixel_sdk/`) — Apache 2.0
- **nlohmann/json** (`drumrobot/lib/nlohmann/json.hpp`) — MIT

### 하드웨어
- USB-CAN 어댑터 (`can0`, `can1` 등으로 인식)
- Dynamixel U2D2 또는 호환 USB-to-TTL (`/dev/ttyUSB0`)

---

## 사전 준비 사항

### 1. CAN 인터페이스
CAN 포트 활성화는 `CanInterface::initialize()`가 자동으로 처리합니다 (sudo 권한 필요).
수동으로 확인하려면:
```bash
ip link show | grep can
```

### 2. Dynamixel USB 포트 권한
```bash
sudo chmod 666 /dev/ttyUSB0
# 또는 영구 설정:
sudo usermod -aG dialout $USER
# (재로그인 필요)
```

### 3. 로그 디렉토리
실행 전 로그 디렉토리를 생성해주세요 (코드에서 자동 생성하지 않습니다):
```bash
mkdir -p drumrobot/log
```

---

## 명령 프로토콜 예시

키보드 모드 또는 TCP 클라이언트에서 다음과 같이 입력합니다:

```
START                           # 시스템 시작 (home 포즈로 이동)
POSE|home                       # home 포즈로 이동
POSE|ready                      # ready 포즈로 이동
LOOK|0|10                       # 정면, 위쪽 10도 응시
LOOK|-30|0                      # 왼쪽 30도 응시
GESTURE|nod                     # 끄덕임
GESTURE|wave                    # 손 흔들기
GESTURE|hurray                  # 환호
MOVE|right_wrist|45             # right_wrist를 45도로 이동 (기본 2.0초)
MOVE|right_wrist|45|1.0         # right_wrist를 45도로 1.0초에 이동
MOVE|waist|-30|2.0              # 허리를 -30도로 2초에 이동
QUIT                            # shutdown 포즈 이동 후 종료
```

각도 단위는 모두 **도(degree)**이며 내부적으로 라디안으로 변환됩니다.

---

## 안전 메커니즘 상세

### TMotor 전류 초과 차단
- 수신된 `current_current`가 `current_limit`을 **연속 5회 초과**하면 시스템 정지
- 일시적 과전류(스파이크)는 카운터가 리셋되어 무시됨
- 카운터 변수: `Motor::cnt`

### IK 실패 처리
- `KinematicsSolver::ik_solve()`가 `success=false` 반환 시
  - 해당 `MotionPrimitive` 폐기
  - 시스템은 **계속 동작** (다음 명령 처리 가능)
- 실패 원인: 도달 불가능한 좌표, NaN/Inf, 관절 한계 초과

### 송신 전 안전 검사 (TMotor만)
- **급변 차단**: 목표 위치와 현재 위치의 차이가 **10도(약 0.175 rad)** 이상이면 해당 모터 송신만 건너뜀
- **범위 검사**: `motors.json`의 `min_angle` / `max_angle` 범위를 벗어나면 송신 건너뜀
- MaxonMotor와 Dynamixel은 송신 전 안전 검사가 별도로 없음 (CSP 모드에서는 모터 자체 제한에 의존)

### 수신 후 안전 검사
- **TMotor 범위 초과** → 즉시 `ctx.running = false` (전체 시스템 정지)
- **Maxon 범위 초과** → `getShutdown` PDO 송신 후 전체 시스템 정지
- 복구 절차:
  1. 시스템 종료 후 모터를 안전 범위로 수동 이동
  2. CAN 포트 재기동 권장 (USB 전원 차단 후 재연결)
  3. `motors.json` 한계 값 검토 후 재실행

---

## 알려진 한계 및 TODO

| 항목 | 상태 | 비고 |
|---|---|---|
| `HIT` (드럼 타격) | ❌ 미구현 | 프레임워크만 존재, `handle_hit`가 로그만 출력 |
| TMotor MIT 모드 | ⚠️ 부분 구현 | 코덱(`TMotorMITCodec`)은 구현됨, Controller에서 미사용 |
| Maxon CSV 모드 | ❌ 미구현 | `Controller::maxon_motor_send_task`에서 에러 출력 |
| IDLE 모션 미세 움직임 | ❌ 미구현 | 현재는 정지 상태 유지만 (`trajectory_generator.cpp` TODO 주석) |
| 다관절 동시 MOVE | ❌ 미구현 | `handle_move`에 TODO 주석 |
| CAN 초기화 전 USB 리셋 | ❌ 미구현 | `robot.cpp` TODO 주석 (USB 전원 껐다 켜기) |
| 모션 로그 상세화 | ❌ 미구현 | `record_motion`에 TODO 주석 |
| Arduino / USBIO 연동 | ❌ 미구현 | `Robot::initialize` TODO 주석 |

---

## 트러블슈팅

### "No CAN port found"
- `ip link show | grep can`로 CAN 포트 존재 여부 확인
- USB-CAN 어댑터 연결 상태 확인
- `dmesg | tail`로 커널 메시지 확인

### "Failed to open the Dynamixel port!"
- `/dev/ttyUSB0` 존재 여부 확인: `ls /dev/ttyUSB*`
- 포트 권한 확인: `sudo chmod 666 /dev/ttyUSB0`
- 다른 프로세스가 점유 중인지 확인: `sudo lsof /dev/ttyUSB0`

### "control_queue underflow"
- MotionPlanner가 ControlQueue에 데이터를 충분히 공급하지 못함
- 원인:
  - CPU 부하 과다
  - IK 실패가 반복되어 trajectory 생성 실패
  - `threshold` 값(`motion_planner.hpp`, 기본 20)이 너무 낮음
- 100회마다 1회 출력되므로 가끔 한두 번은 정상 범위

### "TMotor 급변 차단"
- `desired_joint - current_joint_angle`이 10도 이상일 때 발생
- 발생 조건:
  - 시작 시 로봇이 `init` 자세에 있지 않음 → 수동으로 `init` 자세 맞춘 후 실행
  - 직전 명령 수행 중 외부 충격으로 위치가 크게 어긋남
  - `motors.json`의 `initial_joint_angle`이 실제 영점과 다름

### "Joint X out of range" (IK 실패)
- 목표 좌표가 로봇 작업공간 밖
- `kinematics.json`의 `joint_limits`가 너무 좁게 설정됨
- IK 해는 존재하지만 한계 초과 → 다른 자세를 시도하거나 한계 완화 검토

### "Maxon QuickStop / Shutdown"
- 수신된 위치가 범위 초과
- 발생 시 시스템 전체 정지 → 복구 절차 참조

---

## 좌표계 / 운동학 컨벤션

### 단위
| 위치 | 단위 |
|---|---|
| 사용자 입력 (TCP/키보드) | **도(degree)** |
| JSON 설정 파일 (`motors.json`, `robot_poses.json`, `kinematics.json`) | **도(degree)** |
| 코드 내부 (`q`, `qd`, 모든 관절각) | **라디안(rad)** |
| 모터 위치 (TMotor) | **rad** (모터 좌표계, `direction_sign` 및 `initial_joint_angle`로 변환) |
| 모터 위치 (Maxon) | **rad** (감속비 적용 전) |
| 작업공간 좌표 (`p_target_R`, `p_target_L`) | **미터(m)** |

### 좌표계
- 베이스 원점: 허리(waist) 회전축
- 어깨 간격: `link_length.waist` (0.520 m, kinematics.json 참조)
- 상완 길이: `link_length.upper_arm` (0.230 m)
- 하완 길이: `link_length.forearm` (0.200 m)
- 스틱 길이: `link_length.stick` (0.373 m)

### DH 컨벤션
- **Modified DH (Craig)** 사용
- 변환 행렬: `KinematicsSolver::dh_transform(a, alpha, d, theta)`
- 오른팔 / 왼팔 각각 6단 DH로 손 끝 좌표 계산

### 관절 방향
- 각 관절의 회전 방향은 `motors.json`의 `direction_sign` (±1.0)로 정의
- 모터 좌표 → 관절 좌표: `joint = motor * direction_sign + initial_joint_angle`

---

## 라이선스 및 외부 의존성

> ⚠️ 현재 저장소에 LICENSE 파일이 없습니다. 공개 전 라이선스 명시를 권장합니다.

### 포함된 외부 라이브러리
| 라이브러리 | 위치 | 라이선스 |
|---|---|---|
| nlohmann/json | `drumrobot/lib/nlohmann/json.hpp` | MIT |
| Dynamixel SDK | `drumrobot/lib/dynamixel_sdk/` | Apache 2.0 |

### 기여 가이드 (제안)
- 이슈/PR은 GitHub를 통해 환영합니다
- 새 모터 타입 추가 시: `motor.hpp`/`motor.cpp`에 클래스 추가 + `motor_codec`에 코덱 추가 + `robot.cpp::init_motor_from_json`에 분기 추가
- 새 Opcode 추가 시: `command_parser.hpp`의 `enum class Opcode` + `to_opcode` + `validate_args` + `behavior_planner`의 핸들러
- 코드 스타일: 기존 코드와 일관성 유지 (snake_case, 한국어 주석 허용)

---

## 부록: 빌드 옵션 상세

`drumrobot/Makefile`의 주요 설정:

```makefile
CC      = g++
CFLAGS  = -Wall -O2 -g -std=c++17 -fPIC
INCLUDE = -I./include -I./lib -I./lib/dynamixel_sdk/include
LDFLAGS = -lpthread
```

- `-O2`: 최적화 (실시간 성능 확보)
- `-g`: 디버그 심볼 (gdb 사용 가능)
- `-fPIC`: 위치 독립 코드 (라이브러리 호환성)
- `-lpthread`: POSIX 스레드 (`std::thread`, `SCHED_FIFO`)

디버그 빌드가 필요한 경우 `-O2`를 `-O0`으로 변경하여 사용.