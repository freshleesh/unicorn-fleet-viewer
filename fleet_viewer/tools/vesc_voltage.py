#!/usr/bin/env python3
"""Print the VESC input voltage (V). Stdlib only.

If the ROS VESC driver owns the port, read /vesc/sensors/core instead
(pass the ROS shell setup as argv[1], e.g. "source ~/unicorn_ws/src/unicorn-racing-stack/unicorn.sh").
"""
import glob, os, struct, subprocess, sys, termios, time


def crc16(data):
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def driver_running():
    return subprocess.run(['pgrep', '-f', 'vesc_driver_node'], capture_output=True).returncode == 0


def read_serial():
    ports = glob.glob('/dev/serial/by-id/*ChibiOS*') or glob.glob('/dev/ttyACM*')
    fd = os.open(ports[0], os.O_RDWR | os.O_NOCTTY)
    try:
        attr = termios.tcgetattr(fd)
        attr[0] = attr[1] = attr[3] = 0              # raw
        attr[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        termios.tcsetattr(fd, termios.TCSANOW, attr)
        termios.tcflush(fd, termios.TCIOFLUSH)
        payload = bytes([4])                          # COMM_GET_VALUES
        os.write(fd, bytes([2, len(payload)]) + payload + struct.pack('>H', crc16(payload)) + b'\x03')
        buf, t0 = b'', time.time()
        while time.time() - t0 < 1.0:
            buf += os.read(fd, 512) if select_ready(fd) else b''
            i = buf.find(b'\x02')
            if i >= 0 and len(buf) >= i + 2:
                n = buf[i + 1]
                pkt = buf[i + 2:i + 2 + n]
                if len(pkt) == n and pkt[:1] == b'\x04':
                    return struct.unpack('>h', pkt[27:29])[0] / 10.0
        raise TimeoutError('no reply from VESC')
    finally:
        os.close(fd)


def select_ready(fd):
    import select
    return bool(select.select([fd], [], [], 0.05)[0])


def read_ros(setup):
    cmd = f'{{ {setup}; }} >/dev/null 2>&1; timeout 5 ros2 topic echo --once --field state.voltage_input /vesc/sensors/core'
    out = subprocess.run(['bash', '-c', cmd], capture_output=True, text=True).stdout.split()
    return float(out[0])  # raises if the topic was not received


if __name__ == '__main__':
    try:
        v = read_ros(sys.argv[1]) if driver_running() and len(sys.argv) > 1 else read_serial()
        print(f'{v:.2f}')
    except Exception as e:
        print(f'ERR {type(e).__name__}: {e}')
