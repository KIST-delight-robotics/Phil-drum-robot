# python3 drumrobot_client/main.py
import socket

HOST = '127.0.0.1'
PORT = 1951

def send(s, packet):
    s.sendall((packet + "\n").encode())

# 모드(메뉴) 정의 — 번호: (라벨, 보낼 OPCODE)
MODES = {
    "1": ("연주 모드 (악보)",      lambda: f"PLAY|{input('  악보 이름: ').strip()}"),
    "2": ("제스처 모드",           lambda: f"GESTURE|{input('  종류(nod/shake/wave/hi/hurray/happy): ').strip()}"),
    "3": ("드럼 타격",             lambda: f"HIT|{input('  타겟(snare/ride/bass...): ').strip()}"),
    "4": ("시선 제어",             lambda: f"LOOK|{input('  pan: ').strip()}|{input('  tilt: ').strip()}"),
    "5": ("포즈",                  lambda: f"POSE|{input('  포즈(home/ready/shutdown): ').strip()}"),
}

def main():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((HOST, PORT))
        print(f"Connected to {HOST}:{PORT}")

        # 1단계: START
        while input("'start'를 입력하세요 > ").strip().lower() != "start":
            print("  start 를 입력해야 합니다.")
        send(s, "START")

        # 2단계: READY
        while input("키를 모두 제거한 후 'ready'를 입력하세요 > ").strip().lower() != "ready":
            print("  ready 를 입력해야 합니다.")
        send(s, "READY")
        print("준비 완료.")

        # 3단계: 모드 선택 루프
        while True:
            print("\n=== 모드 선택 ===")
            for k, (label, _) in MODES.items():
                print(f"  {k}. {label}")
            print("  q. 종료")
            choice = input("선택 > ").strip().lower()

            if choice in ("q", "quit"):
                send(s, "QUIT")
                break
            if choice in MODES:
                send(s, MODES[choice][1]())   # 해당 OPCODE 생성 후 전송
            else:
                print("  올바른 번호를 선택하세요.")

if __name__ == "__main__":
    main()