#!/usr/bin/env python3

import os
import sys
import csv
import time
import curses
import socket
import serial
import struct
import threading
import subprocess
from datetime import datetime
from collections import deque

from uisensorsocket import read_valid_frame, parse_frame

# ==============================
# Settings
# ==============================

SERIAL_PORT = "/dev/ttyUSB0"
BAUDRATE = 38400

HOST = "127.0.0.1"
PORT = 8080
LOOP_PERIOD = 0.005

RX_PACKET_SIZE = 29  # from effmea telemetry packet
PAIR_DURATION_S = 20.0

TORQUE_START = -2000
TORQUE_END = 2000
TORQUE_STEP = 200

VEL_START = 0
VEL_END = 1000000
VEL_STEP = 100000

ACT_MODE = 10   # CST
LOAD_MODE = 9   # CSV
SHUTDOWN_MODE = 10
SHUTDOWN_HOLD_S = 0.5
FB_LOG_PERIOD_S = 0.1

STATE_TEXT = {
    0: "RUNNING",
    1: "THERMAL_PAUSED",
    2: "DRIVE_FAULT",
    3: "DRIVE_ERROR",
}

ERROR_TEXT = {
    0: "OK",
    1: "ACTUATOR_FAULT",
    2: "LOAD_FAULT",
    3: "ACTUATOR_DRIVE_ERR",
    4: "LOAD_DRIVE_ERR",
    5: "THERMAL_PAUSED",
}

# ==============================
# Shared State
# ==============================

sensor_lock = threading.Lock()
status_lock = threading.Lock()

latest_sensor_data = {"torque": 0.0, "speed": 0.0}

status = {
    "pair_index": 0,
    "total_pairs": 0,
    "torque": 0,
    "velocity": 0,
    "pair_elapsed_s": 0.0,
    "retry_count": 0,
    "experiment_done": False,
    "thermal_paused": False,
    "state_code": 0,
    "error_code": 0,
    "a_temp": 0,
    "l_temp": 0,
    "log_path": "",
    "rx_frames": 0,
}

sensor_logs = deque(maxlen=300)
actuator_logs = deque(maxlen=300)
load_logs = deque(maxlen=300)

stop_event = threading.Event()


def timestamp():
    return datetime.now().strftime("%H:%M:%S.%f")[:-3]


def log_sensor(msg):
    sensor_logs.append(msg)


def log_act(msg):
    actuator_logs.append(msg)


def log_load(msg):
    load_logs.append(msg)


def clamp_range(start, end, step):
    return list(range(start, end + (1 if step > 0 else -1), step))


class TorqueLogWriter:
    def __init__(self, experiment_dir):
        self.experiment_dir = experiment_dir
        self.csv_file = None
        self.csv_writer = None
        self.current_torque = None
        self.path = ""

    def open_for_torque(self, torque):
        self.close()
        name = f"torque_{torque:+d}.csv"
        self.path = os.path.join(self.experiment_dir, name)
        self.csv_file = open(self.path, "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.current_torque = torque
        self.csv_writer.writerow([
            "timestamp",
            "target_flag",
            "sensor_torque",
            "sensor_rpm",
            "act_target_torque",
            "act_target_velocity",
            "load_target_torque",
            "load_target_velocity",
            "act_torque",
            "act_vel",
            "act_pos",
            "load_torque",
            "load_vel",
            "load_pos",
            "thermal_paused",
        ])
        self.csv_file.flush()

    def write_rows(self, rows):
        if self.csv_writer is not None and rows:
            self.csv_writer.writerows(rows)
            self.csv_file.flush()

    def close(self):
        if self.csv_file is not None:
            self.csv_file.close()
            self.csv_file = None
            self.csv_writer = None
            self.current_torque = None
            self.path = ""


def build_tx_packet(a_torque, a_velocity, a_mode, l_torque, l_velocity, l_mode):
    return struct.pack(
        "!hihhih",
        int(a_torque), int(a_velocity), int(a_mode),
        int(l_torque), int(l_velocity), int(l_mode),
    )


def send_shutdown_commands(conn):
    if conn is None:
        return
    shutdown_packet = build_tx_packet(0, 0, SHUTDOWN_MODE, 0, 0, SHUTDOWN_MODE)
    end_t = time.perf_counter() + SHUTDOWN_HOLD_S
    while time.perf_counter() < end_t:
        try:
            conn.sendall(shutdown_packet)
        except (BrokenPipeError, ConnectionResetError, OSError):
            break
        time.sleep(LOOP_PERIOD)


def serial_thread():
    try:
        ser = serial.Serial(
            port=SERIAL_PORT,
            baudrate=BAUDRATE,
            bytesize=8,
            parity="N",
            stopbits=1,
            timeout=0.01,
        )
        log_sensor("[RS485] Started")

        while not stop_event.is_set():
            frame = read_valid_frame(ser)
            torque, speed, _ = parse_frame(frame)
            with sensor_lock:
                latest_sensor_data["torque"] = torque
                latest_sensor_data["speed"] = speed
            log_sensor(f"Torque: {torque} Nm | Speed: {speed} RPM")
    except Exception as e:
        log_sensor(f"[RS485] Error: {e}")


def socket_thread(ethercat_iface):
    torques = clamp_range(TORQUE_START, TORQUE_END, TORQUE_STEP)
    velocities = clamp_range(VEL_START, VEL_END, VEL_STEP)
    pairs = [(t, v) for t in torques for v in velocities]
    total_pairs = len(pairs)

    with status_lock:
        status["total_pairs"] = total_pairs

    base_dir = os.path.dirname(os.path.abspath(__file__))
    data_dir = os.path.join(base_dir, "../data")
    os.makedirs(data_dir, exist_ok=True)
    session_stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    experiment_dir = os.path.join(data_dir, f"experiment_{session_stamp}")
    os.makedirs(experiment_dir, exist_ok=True)
    writer = TorqueLogWriter(experiment_dir)

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((HOST, PORT))
    server.listen(1)

    proc = subprocess.Popen(
        ["./effmea", ethercat_iface],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )

    conn = None
    try:
        log_act("[TCP] Waiting for effmea...")
        conn, _ = server.accept()
        conn.setblocking(False)
        log_act("[TCP] Connected")

        pair_index = 0
        pair_elapsed_s = 0.0
        retry_count = 0
        pair_interrupted = False
        last_tick = time.perf_counter()
        latest_thermal_paused = False
        prev_pair = None
        prev_state_code = None
        prev_error_code = None
        last_fb_log_t = 0.0
        rx_frames = 0

        if pairs:
            writer.open_for_torque(pairs[0][0])
            with status_lock:
                status["log_path"] = writer.path
            log_act(f"[AUTO] Log dir: {experiment_dir}")
            log_load(f"[AUTO] Log dir: {experiment_dir}")

        rx_buffer = bytearray()
        current_pair_rows = []

        while not stop_event.is_set():
            loop_start = time.perf_counter()

            try:
                data = conn.recv(4096)
                if data:
                    rx_buffer.extend(data)
            except BlockingIOError:
                pass
            except ConnectionResetError:
                log_act("[TCP] Connection reset")
                log_load("[TCP] Connection reset")
                break

            while len(rx_buffer) >= RX_PACKET_SIZE:
                packet = rx_buffer[:RX_PACKET_SIZE]
                del rx_buffer[:RX_PACKET_SIZE]

                a_status = packet[0]
                a_error = packet[1]
                a_temp = packet[2]
                a_tor = struct.unpack(">h", packet[3:5])[0]
                a_vel = struct.unpack(">i", packet[5:9])[0]
                a_pos = struct.unpack(">i", packet[9:13])[0]

                l_status = packet[13]
                l_error = packet[14]
                l_temp = packet[15]
                l_tor = struct.unpack(">h", packet[16:18])[0]
                l_vel = struct.unpack(">i", packet[18:22])[0]
                l_pos = struct.unpack(">i", packet[22:26])[0]

                thermal_paused = bool(packet[26])
                state_code = packet[27]
                error_code = packet[28]
                latest_thermal_paused = thermal_paused
                rx_frames += 1
                if thermal_paused:
                    pair_interrupted = True

                if prev_state_code is None or prev_state_code != state_code:
                    log_act(f"[STATE] {STATE_TEXT.get(state_code, f'UNKNOWN({state_code})')}")
                    log_load(f"[STATE] {STATE_TEXT.get(state_code, f'UNKNOWN({state_code})')}")
                    prev_state_code = state_code

                if prev_error_code is None or prev_error_code != error_code:
                    log_act(f"[ERROR] {ERROR_TEXT.get(error_code, f'UNKNOWN({error_code})')}")
                    log_load(f"[ERROR] {ERROR_TEXT.get(error_code, f'UNKNOWN({error_code})')}")
                    prev_error_code = error_code

                state_text = STATE_TEXT.get(state_code, f"UNKNOWN({state_code})")
                error_text = ERROR_TEXT.get(error_code, f"UNKNOWN({error_code})")

                with status_lock:
                    status["a_temp"] = a_temp
                    status["l_temp"] = l_temp
                    status["thermal_paused"] = thermal_paused
                    status["state_code"] = state_code
                    status["error_code"] = error_code
                    status["rx_frames"] = rx_frames

                if pair_index < total_pairs:
                    cmd_torque, cmd_velocity = pairs[pair_index]
                else:
                    cmd_torque, cmd_velocity = 0, 0

                with sensor_lock:
                    s_torque = latest_sensor_data["torque"]
                    s_speed = latest_sensor_data["speed"]

                if (
                    writer.csv_writer is not None
                    and pair_index < total_pairs
                    and not pair_interrupted
                    and not thermal_paused
                ):
                    current_pair_rows.append([
                        timestamp(),
                        0,
                        s_torque,
                        s_speed,
                        cmd_torque,
                        0,
                        0,
                        cmd_velocity,
                        a_tor,
                        a_vel,
                        a_pos,
                        l_tor,
                        l_vel,
                        l_pos,
                        int(thermal_paused),
                    ])

                now_log_t = time.perf_counter()
                if now_log_t - last_fb_log_t >= FB_LOG_PERIOD_S:
                    log_act(f"FB T={a_temp}C tor={a_tor} vel={a_vel} pos={a_pos}")
                    log_load(f"FB T={l_temp}C tor={l_tor} vel={l_vel} pos={l_pos}")
                    last_fb_log_t = now_log_t

            now = time.perf_counter()
            dt = now - last_tick
            last_tick = now

            if pair_index < total_pairs and not latest_thermal_paused:
                if pair_interrupted:
                    # Thermal pause happened during this pair: discard partial data and restart.
                    current_pair_rows.clear()
                    pair_elapsed_s = 0.0
                    pair_interrupted = False
                    retry_count += 1
                    log_act(f"[AUTO] Pair restarted after thermal pause (retry={retry_count})")
                    log_load(f"[AUTO] Pair restarted after thermal pause (retry={retry_count})")

                pair_elapsed_s += dt
                if pair_elapsed_s >= PAIR_DURATION_S:
                    writer.write_rows(current_pair_rows)
                    current_pair_rows.clear()
                    pair_index += 1
                    pair_elapsed_s = 0.0
                    retry_count = 0
                    if pair_index < total_pairs:
                        next_t, next_v = pairs[pair_index]
                        if writer.current_torque != next_t:
                            writer.open_for_torque(next_t)
                        with status_lock:
                            status["log_path"] = writer.path
                        log_act(f"[AUTO] Next pair torque={next_t}, velocity={next_v}")
                        log_load(f"[AUTO] Next pair torque={next_t}, velocity={next_v}")
                    else:
                        writer.close()
                        with status_lock:
                            status["log_path"] = ""
                            status["experiment_done"] = True
                        log_act("[AUTO] Sweep complete. Holding zero command.")
                        log_load("[AUTO] Sweep complete. Holding zero command.")
            elif pair_index < total_pairs and latest_thermal_paused:
                pair_interrupted = True

            if pair_index < total_pairs:
                target_torque, target_velocity = pairs[pair_index]
                exp_done = False
            else:
                target_torque, target_velocity = 0, 0
                exp_done = True

            with status_lock:
                status["pair_index"] = min(pair_index + 1, total_pairs) if total_pairs else 0
                status["torque"] = target_torque
                status["velocity"] = target_velocity
                status["pair_elapsed_s"] = pair_elapsed_s
                status["retry_count"] = retry_count
                status["experiment_done"] = exp_done

            if prev_pair != (target_torque, target_velocity):
                prev_pair = (target_torque, target_velocity)
                log_act(f"[TX] {target_torque} 0 {ACT_MODE} 0")
                log_load(f"[TX] 0 {target_velocity} {LOAD_MODE} 1")

            tx = build_tx_packet(target_torque, 0, ACT_MODE, 0, target_velocity, LOAD_MODE)
            try:
                conn.sendall(tx)
            except (BrokenPipeError, ConnectionResetError):
                log_act("[TCP] Send failed")
                log_load("[TCP] Send failed")
                break

            elapsed = time.perf_counter() - loop_start
            sleep_s = LOOP_PERIOD - elapsed
            if sleep_s > 0:
                time.sleep(sleep_s)

    finally:
        log_act(f"[TX] Shutdown command: 0 0 {SHUTDOWN_MODE} 0 / 0 0 {SHUTDOWN_MODE} 1")
        log_load(f"[TX] Shutdown command: 0 0 {SHUTDOWN_MODE} 0 / 0 0 {SHUTDOWN_MODE} 1")
        send_shutdown_commands(conn)
        writer.close()
        if conn is not None:
            try:
                conn.close()
            except Exception:
                pass
        try:
            server.close()
        except Exception:
            pass
        try:
            proc.terminate()
            proc.wait(timeout=1.0)
        except Exception:
            pass
        stop_event.set()


def ui_loop(stdscr):
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.timeout(100)

    while not stop_event.is_set():
        stdscr.clear()
        height, width = stdscr.getmaxyx()
        col_w = width // 4

        win_cmd = stdscr.derwin(height, col_w, 0, 0)
        win_sen = stdscr.derwin(height, col_w, 0, col_w)
        win_act = stdscr.derwin(height, col_w, 0, 2 * col_w)
        win_load = stdscr.derwin(height, width - 3 * col_w, 0, 3 * col_w)

        win_cmd.box()
        win_sen.box()
        win_act.box()
        win_load.box()

        win_cmd.addstr(0, 2, " AUTO EXP ")
        win_sen.addstr(0, 2, " TORQUE SENSOR ")
        win_act.addstr(0, 2, " ACTUATOR ")
        win_load.addstr(0, 2, " LOAD ")

        with status_lock:
            pair_idx = status["pair_index"]
            total_pairs = status["total_pairs"]
            tq = status["torque"]
            vel = status["velocity"]
            elapsed_s = status["pair_elapsed_s"]
            retry_count = status["retry_count"]
            exp_done = status["experiment_done"]
            thermal_paused = status["thermal_paused"]
            state_code = status["state_code"]
            error_code = status["error_code"]
            a_temp = status["a_temp"]
            l_temp = status["l_temp"]
            log_path = status["log_path"]
            rx_frames = status["rx_frames"]

        state_text = STATE_TEXT.get(state_code, f"UNKNOWN({state_code})")
        error_text = ERROR_TEXT.get(error_code, f"UNKNOWN({error_code})")
        remain_s = max(0.0, PAIR_DURATION_S - elapsed_s)

        win_cmd.addstr(2, 2, f"Pair: {pair_idx}/{total_pairs}")
        win_cmd.addstr(3, 2, f"Torque: {tq}")
        win_cmd.addstr(4, 2, f"Velocity: {vel}")
        win_cmd.addstr(5, 2, f"Elapsed: {elapsed_s:5.1f}s")
        win_cmd.addstr(6, 2, f"Remain: {remain_s:5.1f}s")
        win_cmd.addstr(7, 2, f"Retry: {retry_count}")
        win_cmd.addstr(8, 2, f"A Temp: {a_temp} C")
        win_cmd.addstr(9, 2, f"L Temp: {l_temp} C")
        win_cmd.addstr(10, 2, f"Thermal: {'PAUSE' if thermal_paused else 'RUN'}")
        win_cmd.addstr(11, 2, f"State: {state_text[:col_w-10]}")
        win_cmd.addstr(12, 2, f"Error: {error_text[:col_w-10]}")
        win_cmd.addstr(13, 2, f"Done: {'YES' if exp_done else 'NO'}")
        win_cmd.addstr(14, 2, f"RX frames: {rx_frames}")

        if log_path:
            base = os.path.basename(log_path)
            win_cmd.addstr(15, 2, f"Log: {base[:max(1, col_w-8)]}")
        win_cmd.addstr(height - 2, 2, "ESC: stop")

        for i, line in enumerate(list(sensor_logs)[-height + 2:]):
            win_sen.addstr(i + 1, 1, line[:col_w - 2])
        for i, line in enumerate(list(actuator_logs)[-height + 2:]):
            win_act.addstr(i + 1, 1, line[:col_w - 2])
        for i, line in enumerate(list(load_logs)[-height + 2:]):
            win_load.addstr(i + 1, 1, line[:max(1, width - 3 * col_w - 2)])

        stdscr.refresh()

        key = stdscr.getch()
        if key == 27:
            stop_event.set()
            break


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 auto_exp.py <ethercat_interface>")
        return 1

    ethercat_iface = sys.argv[1]

    serial_t = threading.Thread(target=serial_thread, daemon=True)
    socket_t = threading.Thread(target=socket_thread, args=(ethercat_iface,), daemon=False)

    serial_t.start()
    socket_t.start()

    try:
        curses.wrapper(ui_loop)
    finally:
        stop_event.set()
        socket_t.join(timeout=3.0)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
