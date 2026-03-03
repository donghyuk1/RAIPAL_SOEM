# newsensorsocket.py

import serial
import serial.rs485
import socket
import struct
import subprocess
import threading
import time
import curses
from collections import deque

# ==============================
# 설정
# ==============================

SERIAL_PORT = "/dev/ttyUSB0"
BAUDRATE = 38400
HOST = "127.0.0.1"
PORT = 8080
LOOP_PERIOD = 0.00

# ==============================
# 공유 데이터
# ==============================

sensor_lock = threading.Lock()
control_lock = threading.Lock()

latest_sensor_data = {"torque": 0, "speed": 0}

user_command = {"target_torque": 0, "target_velocity": 0, "mode": 10}
user_ready = False

python_logs = deque(maxlen=200)
cpp_logs = deque(maxlen=200)

# ==============================
# 로그 함수
# ==============================

def log_py(msg):
    python_logs.append(msg)

def log_cpp(msg):
    cpp_logs.append(msg)

# ==============================
# RS485 Thread
# ==============================

def serial_thread():
    try:
        ser = serial.Serial(
            port=SERIAL_PORT,
            baudrate=BAUDRATE,
            bytesize=8,
            parity='N',
            stopbits=1,
            timeout=0.01
        )
        ser.rs485_mode = serial.rs485.RS485Settings()
        log_py("[RS485] Started")

        while True:
            data = ser.read(6)
            if len(data) != 6:
                continue

            torque = data[0]
            speed = data[1]

            with sensor_lock:
                latest_sensor_data["torque"] = torque
                latest_sensor_data["speed"] = speed

            log_py(f"[RS485] T={torque} S={speed}")

    except Exception as e:
        log_py(f"[RS485] Error: {e}")

# ==============================
# C++ stdout reader
# ==============================

def cpp_reader_thread(proc):
    for line in proc.stdout:
        log_cpp(line.strip())

# ==============================
# TCP Thread
# ==============================

def socket_thread():

    global user_ready

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((HOST, PORT))
    server.listen(1)

    # effmea 실행 + stdout capture
    proc = subprocess.Popen(
        ["./effmea", "eth0"],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True
    )

    threading.Thread(target=cpp_reader_thread, args=(proc,), daemon=True).start()

    log_py("[TCP] Waiting for effmea...")
    conn, addr = server.accept()
    conn.setblocking(False)
    log_py("[TCP] Connected")

    rx_buffer = bytearray()

    while True:
        start = time.perf_counter()

        # receive
        try:
            data = conn.recv(1024)
            if data:
                rx_buffer.extend(data)
        except BlockingIOError:
            pass

        while len(rx_buffer) >= 12:
            packet = rx_buffer[:12]
            del rx_buffer[:12]

            status = packet[0]
            log_py(f"[TCP RX] status={status}")

        # send
        with control_lock:
            if user_ready:
                t = user_command["target_torque"]
                v = user_command["target_velocity"]
                m = user_command["mode"]
            else:
                t, v, m = 0, 0, 10

        try:
            tx = struct.pack("!hih", t, v, m)
            conn.send(tx)
        except:
            break

        # 200Hz 유지
        elapsed = time.perf_counter() - start
        sleep = LOOP_PERIOD - elapsed
        if sleep > 0:
            time.sleep(sleep)

# ==============================
# UI (curses)
# ==============================

def ui_loop(stdscr):

    global user_ready

    curses.curs_set(1)
    stdscr.nodelay(True)
    stdscr.timeout(50)

    input_buffer = ""

    while True:

        stdscr.clear()
        height, width = stdscr.getmaxyx()

        col_w = width // 3

        # 3개 창
        win_input = stdscr.derwin(height, col_w, 0, 0)
        win_py    = stdscr.derwin(height, col_w, 0, col_w)
        win_cpp   = stdscr.derwin(height, width - 2*col_w, 0, 2*col_w)

        # INPUT 창
        win_input.box()
        win_input.addstr(0, 2, " INPUT ")

        with control_lock:
            win_input.addstr(2, 2, f"Torque: {user_command['target_torque']}")
            win_input.addstr(3, 2, f"Vel   : {user_command['target_velocity']}")
            win_input.addstr(4, 2, f"Mode  : {user_command['mode']}")
            win_input.addstr(6, 2, f"Ready : {user_ready}")

        win_input.addstr(height-2, 2, f"> {input_buffer}")

        # PYTHON LOG
        win_py.box()
        win_py.addstr(0, 2, " PYTHON ")

        for i, line in enumerate(list(python_logs)[-height+2:]):
            win_py.addstr(i+1, 1, line[:col_w-2])

        # CPP LOG
        win_cpp.box()
        win_cpp.addstr(0, 2, " C++ ")

        for i, line in enumerate(list(cpp_logs)[-height+2:]):
            win_cpp.addstr(i+1, 1, line[:col_w-2])

        stdscr.refresh()

        # 키 입력 처리
        try:
            key = stdscr.getch()

            if key == -1:
                continue

            elif key in (10, 13):  # Enter
                parts = input_buffer.strip().split()
                if len(parts) >= 2:
                    with control_lock:
                        user_command["target_torque"] = int(parts[0])
                        user_command["target_velocity"] = int(parts[1])
                        user_command["mode"] = int(parts[2]) if len(parts) > 2 else 10
                        user_ready = True
                input_buffer = ""

            elif key == 27:  # ESC 종료
                break

            elif key == curses.KEY_BACKSPACE or key == 127:
                input_buffer = input_buffer[:-1]

            else:
                input_buffer += chr(key)

        except:
            pass

# ==============================
# MAIN
# ==============================

if __name__ == "__main__":

    threading.Thread(target=serial_thread, daemon=True).start()
    threading.Thread(target=socket_thread, daemon=True).start()

    curses.wrapper(ui_loop)