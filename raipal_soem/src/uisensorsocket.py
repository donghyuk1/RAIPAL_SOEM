# uisensorsocket.py

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
LOOP_PERIOD = 0.005  # 200Hz

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
        # ser.rs485_mode = serial.rs485.RS485Settings()
        log_py("[RS485] Started")

        while True:
            frame = read_valid_frame(ser)
            torque, speed, crc = parse_frame(frame)

            with sensor_lock:
                latest_sensor_data["torque"] = torque
                latest_sensor_data["speed"] = speed

                log_py(f"Torque: {torque} Nm | Speed: {speed} RPM")

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
        ["./effmea", "enxf8e43b090bc0"],
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
            error  = packet[1]
            torque = struct.unpack(">h", packet[2:4])[0]
            velocity = struct.unpack(">i", packet[4:8])[0]
            position = struct.unpack(">i", packet[8:12])[0]

            # log_py(f"status={status} error={error} torque={torque} velocity={velocity} position={position}")

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
        sleep = 0.005 - elapsed
        if sleep > 0:
            time.sleep(sleep)
        else:
            time.sleep(0.001)  # 안전장치

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