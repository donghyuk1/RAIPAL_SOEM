# uisensorsocket.py

import os
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
ETHERCAT_IFACE = None
RX_PACKET_SIZE = 29
# ==============================
# 공유 데이터
# ==============================

sensor_lock = threading.Lock()
control_lock = threading.Lock()
feedback_lock = threading.Lock()

latest_sensor_data = {"torque": 0, "speed": 0}
latest_feedback_state = {
    "a_temp": 0,
    "l_temp": 0,
    "thermal_paused": False,
    "state_code": 0,
    "error_code": 0,
}

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
    sensor_logs.append(msg)

def log_act(msg):
    actuator_logs.append(msg)

def log_load(msg):
    load_logs.append(msg)

# ==============================
# CRC 함수 (네 기존 것 사용)
# ==============================

crc16_table = [0x0000, 0xc0c1, 0xc181, 0x0140, 0xc301, 0x03c0, 0x0280, 0xc241, 
	0xc601, 0x06c0, 0x0780, 0xc741, 0x0500, 0xc5c1, 0xc481, 0x0440, 
	0xcc01, 0x0cc0, 0x0d80, 0xcd41, 0x0f00, 0xcfc1, 0xce81, 0x0e40, 
	0x0a00, 0xcac1, 0xcb81, 0x0b40, 0xc901, 0x09c0, 0x0880, 0xc841, 
	0xd801, 0x18c0, 0x1980, 0xd941, 0x1b00, 0xdbc1, 0xda81, 0x1a40, 
	0x1e00, 0xdec1, 0xdf81, 0x1f40, 0xdd01, 0x1dc0, 0x1c80, 0xdc41, 
	0x1400, 0xd4c1, 0xd581, 0x1540, 0xd701, 0x17c0, 0x1680, 0xd641, 
	0xd201, 0x12c0, 0x1380, 0xd341, 0x1100, 0xd1c1, 0xd081, 0x1040, 
	0xf001, 0x30c0, 0x3180, 0xf141, 0x3300, 0xf3c1, 0xf281, 0x3240, 
	0x3600, 0xf6c1, 0xf781, 0x3740, 0xf501, 0x35c0, 0x3480, 0xf441, 
	0x3c00, 0xfcc1, 0xfd81, 0x3d40, 0xff01, 0x3fc0, 0x3e80, 0xfe41, 
	0xfa01, 0x3ac0, 0x3b80, 0xfb41, 0x3900, 0xf9c1, 0xf881, 0x3840, 
	0x2800, 0xe8c1, 0xe981, 0x2940, 0xeb01, 0x2bc0, 0x2a80, 0xea41, 
	0xee01, 0x2ec0, 0x2f80, 0xef41, 0x2d00, 0xedc1, 0xec81, 0x2c40, 
	0xe401, 0x24c0, 0x2580, 0xe541, 0x2700, 0xe7c1, 0xe681, 0x2640, 
	0x2200, 0xe2c1, 0xe381, 0x2340, 0xe101, 0x21c0, 0x2080, 0xe041, 
	0xa001, 0x60c0, 0x6180, 0xa141, 0x6300, 0xa3c1, 0xa281, 0x6240, 
	0x6600, 0xa6c1, 0xa781, 0x6740, 0xa501, 0x65c0, 0x6480, 0xa441, 
	0x6c00, 0xacc1, 0xad81, 0x6d40, 0xaf01, 0x6fc0, 0x6e80, 0xae41, 
	0xaa01, 0x6ac0, 0x6b80, 0xab41, 0x6900, 0xa9c1, 0xa881, 0x6840, 
	0x7800, 0xb8c1, 0xb981, 0x7940, 0xbb01, 0x7bc0, 0x7a80, 0xba41, 
	0xbe01, 0x7ec0, 0x7f80, 0xbf41, 0x7d00, 0xbdc1, 0xbc81, 0x7c40, 
	0xb401, 0x74c0, 0x7580, 0xb541, 0x7700, 0xb7c1, 0xb681, 0x7640, 
	0x7200, 0xb2c1, 0xb381, 0x7340, 0xb101, 0x71c0, 0x7080, 0xb041, 
	0x5000, 0x90c1, 0x9181, 0x5140, 0x9301, 0x53c0, 0x5280, 0x9241, 
	0x9601, 0x56c0, 0x5780, 0x9741, 0x5500, 0x95c1, 0x9481, 0x5440, 
	0x9c01, 0x5cc0, 0x5d80, 0x9d41, 0x5f00, 0x9fc1, 0x9e81, 0x5e40, 
	0x5a00, 0x9ac1, 0x9b81, 0x5b40, 0x9901, 0x59c0, 0x5880, 0x9841, 
	0x8801, 0x48c0, 0x4980, 0x8941, 0x4b00, 0x8bc1, 0x8a81, 0x4a40, 
	0x4e00, 0x8ec1, 0x8f81, 0x4f40, 0x8d01, 0x4dc0, 0x4c80, 0x8c41, 
	0x4400, 0x84c1, 0x8581, 0x4540, 0x8701, 0x47c0, 0x4680, 0x8641, 
	0x8201, 0x42c0, 0x4380, 0x8341, 0x4100, 0x81c1, 0x8081, 0x4040]

def crc16_modbus(init_crc, dat, len):
	crc = [init_crc >> 8, init_crc & 0xFF]    
	for b in dat:
		tmp = crc16_table[crc[0] ^ b]
		crc[0] = (tmp & 0xFF) ^ crc[1]
		crc[1] = tmp>>8
	
	return (crc[0]<<8|crc[1])

torque_decimal = 1

def sync_to_valid_frame(ser, frame_size=6):
	buffer = ser.read(12)
	#print(buffer)
	for i in range(0, 8):   # 한 바이트씩 이동하여 동기화 시도
		#print(i)
		frame = buffer[i:i+frame_size]
		crc = frame[4] << 8 | frame[5]
		dat = bytes(frame[:4])
		#print(crc, crc16_modbus(0xFFFF, dat, len(dat)))
		if crc == crc16_modbus(0xFFFF, dat, len(dat)):
			return i

def read_frame(ser):
	"""RS485에서 6바이트 프레임 읽기"""
	while True:
		data = ser.read(6)
		if len(data) != 6:
			continue
		return data
		
def read_valid_frame(ser, frame_size=6):
	buffer = bytearray()
	while True:
		buffer += ser.read(6)
		while len(buffer) >= frame_size:
			frame = buffer[:frame_size]
			crc = frame[4] << 8 | frame[5]
			dat = frame[:4]
			if crc == crc16_modbus(0xFFFF, dat, len(dat)):
				buffer = buffer[frame_size:]  # remove processed frame
				return frame
			else:
				buffer = buffer[1:]  # shift 1 byte and resync


def parse_frame(frame):
	"""데이터(D1~D6) 파싱"""
	torque_raw = frame[0] << 8 | frame[1]
	torque_raw = round(torque_raw*(0.1**torque_decimal), torque_decimal)
	speed_raw = (frame[2] & 0x7F) << 8 | frame[3]
	crc = frame[4] << 8 | frame[5]
	# D3 최상위 비트(토크 부호)
	torque_sign = 1 if frame[2] & 0x80 else 0
	torque = -torque_raw if torque_sign else torque_raw
	return torque, speed_raw, crc


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
        log_sensor("[RS485] Started")

        while True:
            frame = read_valid_frame(ser)
            torque, speed, crc = parse_frame(frame)

            with sensor_lock:
                latest_sensor_data["torque"] = torque
                latest_sensor_data["speed"] = speed

            log_sensor(f"Torque: {torque} Nm | Speed: {speed} RPM")

    except Exception as e:
        log_sensor(f"[RS485] Error: {e}")

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
        ["./effmea", ETHERCAT_IFACE],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL
    )

    conn, _ = server.accept()
    conn.setblocking(False)
    log_act("[TCP] Connected")

    rx_buffer = bytearray()
    prev_thermal_paused = None
    prev_state_code = None
    prev_error_code = None

    while True:
        start = time.perf_counter()

        # ---------- RECEIVE ----------
        try:
            data = conn.recv(1024)
            if data:
                rx_buffer.extend(data)
        except BlockingIOError:
            pass

        while len(rx_buffer) >= RX_PACKET_SIZE:
            packet = rx_buffer[:RX_PACKET_SIZE]
            del rx_buffer[:RX_PACKET_SIZE]

            a_tor = struct.unpack(">h", packet[3:5])[0]
            a_vel = struct.unpack(">i", packet[5:9])[0]
            a_pos = struct.unpack(">i", packet[9:13])[0]
            a_temp = packet[2]

            l_tor = struct.unpack(">h", packet[16:18])[0]
            l_vel = struct.unpack(">i", packet[18:22])[0]
            l_pos = struct.unpack(">i", packet[22:26])[0]
            l_temp = packet[15]
            thermal_paused = bool(packet[26])
            state_code = packet[27]
            error_code = packet[28]

            with feedback_lock:
                latest_feedback_state["a_temp"] = a_temp
                latest_feedback_state["l_temp"] = l_temp
                latest_feedback_state["thermal_paused"] = thermal_paused
                latest_feedback_state["state_code"] = state_code
                latest_feedback_state["error_code"] = error_code

            ts = timestamp()

            log_act(f"FB T={a_temp}C tor={a_tor} vel={a_vel} pos={a_pos}")
            log_load(f"FB T={l_temp}C tor={l_tor} vel={l_vel} pos={l_pos}")

            if prev_thermal_paused is None or prev_thermal_paused != thermal_paused:
                state = "PAUSED" if thermal_paused else "RUNNING"
                log_act(f"[THERMAL] {state}")
                log_load(f"[THERMAL] {state}")
                prev_thermal_paused = thermal_paused

            if prev_state_code is None or prev_state_code != state_code:
                state_text = STATE_TEXT.get(state_code, f"UNKNOWN({state_code})")
                log_act(f"[STATE] {state_text}")
                log_load(f"[STATE] {state_text}")
                prev_state_code = state_code

            if prev_error_code is None or prev_error_code != error_code:
                error_text = ERROR_TEXT.get(error_code, f"UNKNOWN({error_code})")
                log_act(f"[ERROR] {error_text}")
                log_load(f"[ERROR] {error_text}")
                prev_error_code = error_code

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
                0,

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
            conn.sendall(tx)
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

        with feedback_lock:
            a_temp = latest_feedback_state["a_temp"]
            l_temp = latest_feedback_state["l_temp"]
            thermal_paused = latest_feedback_state["thermal_paused"]
            state_code = latest_feedback_state["state_code"]
            error_code = latest_feedback_state["error_code"]

        if height > 13:
            win_cmd.addstr(10, 2, f"A Temp: {a_temp} C")
            win_cmd.addstr(11, 2, f"L Temp: {l_temp} C")
            win_cmd.addstr(12, 2, f"Thermal: {'PAUSE' if thermal_paused else 'RUN'}")
        if height > 15:
            win_cmd.addstr(13, 2, f"State: {STATE_TEXT.get(state_code, state_code)}")
            win_cmd.addstr(14, 2, f"Error: {ERROR_TEXT.get(error_code, error_code)}")

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

                        # 값 복사 (lock 안에서 한 번만)
                        act_t_target = user_command["actuator"]["torque"]
                        act_v_target = user_command["actuator"]["velocity"]
                        load_t_target = user_command["load"]["torque"]
                        load_v_target = user_command["load"]["velocity"]

                    # sensor 읽기
                    with sensor_lock:
                        s_torque = latest_sensor_data["torque"]
                        s_speed  = latest_sensor_data["speed"]

                    # CSV 기록
                    ts = timestamp()

                    csv_writer.writerow([
                        ts,
                        1,
                        s_torque, s_speed,
                        act_t_target, act_v_target,
                        load_t_target, load_v_target,
                        "", "", "",
                        "", "", ""
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

# ==============================
# CSV 파일 경로 생성
# ==============================

    BASE_DIR = os.path.dirname(os.path.abspath(__file__))
    DATA_DIR = os.path.join(BASE_DIR, "../data")

    os.makedirs(DATA_DIR, exist_ok=True)

    start_time_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = os.path.join(DATA_DIR, f"experiment_{start_time_str}.csv")

    csv_file = open(csv_path, "w", newline="")
    csv_writer = csv.writer(csv_file)

    print(f"[LOG] Saving data to: {csv_path}")
    csv_writer.writerow([
    "timestamp",
    "target_flag",

    "sensor_torque", "sensor_rpm",

    "act_target_torque", "act_target_velocity",
    "load_target_torque", "load_target_velocity",

    "act_torque","act_vel","act_pos",
    "load_torque","load_vel","load_pos"
    ])

    if len(sys.argv) < 2:
        print("Usage: python3 uisensorsocket.py <ethercat_interface>")
        sys.exit(1)

    ETHERCAT_IFACE = sys.argv[1]

    threading.Thread(target=serial_thread, daemon=True).start()
    threading.Thread(target=socket_thread, daemon=True).start()

    curses.wrapper(ui_loop)
