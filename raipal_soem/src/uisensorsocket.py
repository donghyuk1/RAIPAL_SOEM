import sys
import serial
import socket
import struct
import subprocess
import threading
import time
import curses
from collections import deque
import csv
from datetime import datetime

# ==============================
# 설정
# ==============================

SERIAL_PORT = "/dev/ttyUSB0"
BAUDRATE = 38400
HOST = "127.0.0.1"
PORT = 8080
LOOP_PERIOD = 0.005
ETHERCAT_IFACE_1 = None
ETHERCAT_IFACE_2 = None

# ==============================
# 공유 데이터
# ==============================

sensor_lock = threading.Lock()
control_lock = threading.Lock()

latest_sensor_data = {"torque": 0, "speed": 0}

user_command = {
    "actuator": {"torque": 0, "velocity": 0, "mode": 10},
    "load":     {"torque": 0, "velocity": 0, "mode": 10}
}

user_ready = False

sensor_logs   = deque(maxlen=200)
actuator_logs = deque(maxlen=200)
load_logs     = deque(maxlen=200)

# ==============================
# 로그 함수
# ==============================

def timestamp():
    return datetime.now().strftime("%H:%M:%S.%f")[:-3]

def log_sensor(msg):
    sensor_logs.append(f"{timestamp()} {msg}")

def log_act(msg):
    actuator_logs.append(f"{timestamp()} {msg}")

def log_load(msg):
    load_logs.append(f"{timestamp()} {msg}")

# ==============================
# RS485 Thread
# ==============================

def serial_thread():
    try:
        ser = serial.Serial(
            port=SERIAL_PORT,
            baudrate=BAUDRATE,
            timeout=0.01
        )
        log_sensor("[RS485] Started")

        while True:
            data = ser.read(6)
            if len(data) != 6:
                continue

            torque = data[0] << 8 | data[1]
            speed  = data[2] << 8 | data[3]

            with sensor_lock:
                latest_sensor_data["torque"] = torque
                latest_sensor_data["speed"] = speed

            log_sensor(f"Torque={torque} Speed={speed}")

    except Exception as e:
        log_sensor(f"[RS485] Error: {e}")

# ==============================
# C++ stdout reader
# ==============================

def cpp_reader_thread(proc):
    for line in proc.stdout:
        log_act(line.strip())

# ==============================
# TCP Thread
# ==============================

def socket_thread():

    global user_ready
    global ETHERCAT_IFACE_1, ETHERCAT_IFACE_2

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((HOST, PORT))
    server.listen(1)

    proc = subprocess.Popen(
        ["./effmea", ETHERCAT_IFACE_1, ETHERCAT_IFACE_2],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True
    )

    threading.Thread(target=cpp_reader_thread, args=(proc,), daemon=True).start()

    conn, _ = server.accept()
    conn.setblocking(False)
    log_act("[TCP] Connected")

    rx_buffer = bytearray()

    while True:
        start = time.perf_counter()

        # ---------- RECEIVE ----------
        try:
            data = conn.recv(1024)
            if data:
                rx_buffer.extend(data)
        except BlockingIOError:
            pass

        while len(rx_buffer) >= 24:
            packet = rx_buffer[:24]
            del rx_buffer[:24]

            a_tor = struct.unpack(">h", packet[2:4])[0]
            a_vel = struct.unpack(">i", packet[4:8])[0]
            a_pos = struct.unpack(">i", packet[8:12])[0]

            l_tor = struct.unpack(">h", packet[14:16])[0]
            l_vel = struct.unpack(">i", packet[16:20])[0]
            l_pos = struct.unpack(">i", packet[20:24])[0]

            ts = timestamp()

            log_act(f"FB tor={a_tor} vel={a_vel} pos={a_pos}")
            log_load(f"FB tor={l_tor} vel={l_vel} pos={l_pos}")

            with sensor_lock:
                s_torque = latest_sensor_data["torque"]
                s_speed  = latest_sensor_data["speed"]

            with control_lock:
                act_t_target = user_command["actuator"]["torque"]
                act_v_target = user_command["actuator"]["velocity"]
                load_t_target = user_command["load"]["torque"]
                load_v_target = user_command["load"]["velocity"]

            csv_writer.writerow([
                ts,

                s_torque, s_speed,

                act_t_target, act_v_target,
                load_t_target, load_v_target,

                a_tor, a_vel, a_pos,
                l_tor, l_vel, l_pos
            ])
            csv_file.flush()

        # ---------- SEND ----------
        with control_lock:
            a = user_command["actuator"]
            l = user_command["load"]

        try:
            tx = struct.pack("!hihhih",
                             a["torque"], a["velocity"], a["mode"],
                             l["torque"], l["velocity"], l["mode"])
            conn.send(tx)
        except:
            break

        elapsed = time.perf_counter() - start
        sleep = LOOP_PERIOD - elapsed
        if sleep > 0:
            time.sleep(sleep)

# ==============================
# UI (4열)
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
        col_w = width // 4

        win_cmd  = stdscr.derwin(height, col_w, 0, 0)
        win_sen  = stdscr.derwin(height, col_w, 0, col_w)
        win_act  = stdscr.derwin(height, col_w, 0, 2*col_w)
        win_load = stdscr.derwin(height, width-3*col_w, 0, 3*col_w)

        # ----- COMMAND -----
        win_cmd.box()
        win_cmd.addstr(0, 2, " COMMAND ")

        with control_lock:
            win_cmd.addstr(2, 2, f"A Tor: {user_command['actuator']['torque']}")
            win_cmd.addstr(3, 2, f"A Vel: {user_command['actuator']['velocity']}")
            win_cmd.addstr(4, 2, f"A Mode:{user_command['actuator']['mode']}")
            win_cmd.addstr(6, 2, f"L Tor: {user_command['load']['torque']}")
            win_cmd.addstr(7, 2, f"L Vel: {user_command['load']['velocity']}")
            win_cmd.addstr(8, 2, f"L Mode:{user_command['load']['mode']}")

        win_cmd.addstr(height-2, 2, f"> {input_buffer}")

        # ----- SENSOR -----
        win_sen.box()
        win_sen.addstr(0, 2, " TORQUE SENSOR ")
        for i, line in enumerate(list(sensor_logs)[-height+2:]):
            win_sen.addstr(i+1, 1, line[:col_w-2])

        # ----- ACTUATOR -----
        win_act.box()
        win_act.addstr(0, 2, " ACTUATOR ")
        for i, line in enumerate(list(actuator_logs)[-height+2:]):
            win_act.addstr(i+1, 1, line[:col_w-2])

        # ----- LOAD -----
        win_load.box()
        win_load.addstr(0, 2, " LOAD ")
        for i, line in enumerate(list(load_logs)[-height+2:]):
            win_load.addstr(i+1, 1, line[:col_w-2])

        stdscr.refresh()

        # ---------- INPUT ----------
        try:
            key = stdscr.getch()

            if key == -1:
                continue

            elif key in (10, 13):
                parts = input_buffer.strip().split()
                if len(parts) == 4:
                    torque = int(parts[0])
                    vel    = int(parts[1])
                    mode   = int(parts[2])
                    flag   = int(parts[3])

                    with control_lock:
                        if flag == 0:
                            user_command["actuator"] = {
                                "torque": torque,
                                "velocity": vel,
                                "mode": mode
                            }
                            log_act(f"CMD set to {torque},{vel},{mode}")
                        elif flag == 1:
                            user_command["load"] = {
                                "torque": torque,
                                "velocity": vel,
                                "mode": mode
                            }
                            log_load(f"CMD set to {torque},{vel},{mode}")

                        user_ready = True

                        # ===== target 변경 이벤트를 CSV에 기록 =====
                        ts = timestamp()

                        with sensor_lock:
                            s_torque = latest_sensor_data["torque"]
                            s_speed  = latest_sensor_data["speed"]

                        with control_lock:
                            act_t_target = user_command["actuator"]["torque"]
                            act_v_target = user_command["actuator"]["velocity"]
                            load_t_target = user_command["load"]["torque"]
                            load_v_target = user_command["load"]["velocity"]

                        csv_writer.writerow([
                            ts,

                            s_torque, s_speed,

                            act_t_target, act_v_target,
                            load_t_target, load_v_target,

                            "", "", "",   # actuator feedback 없음
                            "", "", ""    # load feedback 없음
                        ])
                        csv_file.flush()
                input_buffer = ""

            elif key == 27:
                break

            elif key in (curses.KEY_BACKSPACE, 127):
                input_buffer = input_buffer[:-1]

            else:
                input_buffer += chr(key)

        except:
            pass

# ==============================
# MAIN
# ==============================

if __name__ == "__main__":

    csv_file = open("data.csv", "w", newline="")
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow([
    "timestamp",

    "sensor_torque", "sensor_rpm",

    "act_target_torque", "act_target_velocity",
    "load_target_torque", "load_target_velocity",

    "act_torque","act_vel","act_pos",
    "load_torque","load_vel","load_pos"
    ])

    if len(sys.argv) < 3:
        print("Usage: python3 uisensorsocket.py <ethercat_interface_1> <ethercat_interface_2>")
        sys.exit(1)

    ETHERCAT_IFACE_1 = sys.argv[1]
    ETHERCAT_IFACE_2 = sys.argv[2]

    threading.Thread(target=serial_thread, daemon=True).start()
    threading.Thread(target=socket_thread, daemon=True).start()

    curses.wrapper(ui_loop)