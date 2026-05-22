# python3 planner/main.py

import socket

HOST = '127.0.0.1'
PORT = 1951

def main():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((HOST, PORT))
        print(f"Connected to {HOST}:{PORT}")
        print("Type a message and press Enter to send. (quit to exit)")

        while True:
            msg = input("> ").strip()
            if not msg:
                continue
 
            s.sendall(msg.encode())
 
            if msg == "quit" or msg == "q":
                break

if __name__ == "__main__":
    main()