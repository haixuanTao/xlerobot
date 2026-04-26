"""Direct serial control of the WowRobo v2 differential-drive base (Feetech STS3215).

Assumes both wheel motors are configured: left = ID 7, right = ID 8,
baudrate 1_000_000, wheel mode. Both share the same serial bus.

Run: python wheel_control.py /dev/tty.usbmodemXXXX

Keys: w / S = forward / reverse | a / d = turn left / right |
      space = stop | [ / ] = decrease / increase step | q = quit.
"""

import argparse
import sys
import time

import serial

INST_PING = 0x01
INST_READ = 0x02
INST_WRITE = 0x03

ADDR_MAX_TORQUE = 16          # EEPROM, 2 bytes, 0-1000 (1000 = 100% rated)
ADDR_TORQUE_ENABLE = 40       # RAM, 1 byte
ADDR_GOAL_SPEED = 46          # RAM, 2 bytes, signed via bit 15
ADDR_TORQUE_LIMIT = 48        # RAM, 2 bytes, 0-1000
ADDR_LOCK = 55                # EEPROM lock: 0 = unlocked, 1 = locked
ADDR_PRESENT_SPEED = 58

LEFT_ID = 7
RIGHT_ID = 8
BAUDRATE = 1_000_000
SPEED_LIMIT = 1023
TORQUE_LIMIT_MAX = 1000


def checksum(body):
    return (~sum(body)) & 0xFF


def build_packet(motor_id, instruction, params=()):
    length = len(params) + 2
    body = [motor_id, length, instruction, *params]
    return bytes([0xFF, 0xFF, *body, checksum(body)])


def read_response(ser, expected_params=0, timeout=0.05):
    deadline = time.monotonic() + timeout
    buf = bytearray()
    needed = 6 + expected_params
    while time.monotonic() < deadline and len(buf) < needed:
        chunk = ser.read(needed - len(buf))
        if chunk:
            buf.extend(chunk)
    if len(buf) < needed or buf[0] != 0xFF or buf[1] != 0xFF:
        return None
    return bytes(buf)


class WheelMotor:
    def __init__(self, ser, motor_id):
        self.ser = ser
        self.id = motor_id

    def _send(self, instruction, params=()):
        self.ser.reset_input_buffer()
        self.ser.write(build_packet(self.id, instruction, params))
        self.ser.flush()

    def ping(self):
        self._send(INST_PING)
        return read_response(self.ser, expected_params=0) is not None

    def write_bytes(self, address, data):
        self._send(INST_WRITE, [address, *data])
        read_response(self.ser, expected_params=0)

    def read_bytes(self, address, size):
        self._send(INST_READ, [address, size])
        resp = read_response(self.ser, expected_params=size)
        if resp is None:
            return None
        return resp[5 : 5 + size]

    def set_speed(self, speed):
        speed = max(-SPEED_LIMIT, min(SPEED_LIMIT, int(speed)))
        raw = (-speed) | 0x8000 if speed < 0 else speed
        self.write_bytes(ADDR_GOAL_SPEED, [raw & 0xFF, (raw >> 8) & 0xFF])

    def stop(self):
        self.set_speed(0)

    def _write_u16(self, address, value):
        value = max(0, min(0xFFFF, int(value)))
        self.write_bytes(address, [value & 0xFF, (value >> 8) & 0xFF])

    def set_max_torque(self, value):
        """EEPROM register 16. Persists across power cycles. 0-1000."""
        value = max(0, min(TORQUE_LIMIT_MAX, int(value)))
        self.write_bytes(ADDR_LOCK, [0])
        time.sleep(0.02)
        self._write_u16(ADDR_MAX_TORQUE, value)
        time.sleep(0.02)
        self.write_bytes(ADDR_LOCK, [1])

    def set_torque_limit(self, value):
        """RAM register 48. Resets to Max Torque on power-on. 0-1000."""
        value = max(0, min(TORQUE_LIMIT_MAX, int(value)))
        self._write_u16(ADDR_TORQUE_LIMIT, value)

    def set_torque_enable(self, enabled):
        self.write_bytes(ADDR_TORQUE_ENABLE, [1 if enabled else 0])


class DiffDrive:
    def __init__(self, ser, left_id=LEFT_ID, right_id=RIGHT_ID, invert_right=True):
        # `ser` is a pre-opened serial port (pyserial OR xoq.serial.Serial) — the
        # caller decides the transport so we don't depend on serial.Serial being
        # monkey-patched.
        self.ser = ser
        self.left = WheelMotor(self.ser, left_id)
        self.right = WheelMotor(self.ser, right_id)
        self.invert_right = invert_right

    def ping(self):
        return {"left": self.left.ping(), "right": self.right.ping()}

    def apply_torque(self, max_torque=None, torque_limit=None):
        for motor in (self.left, self.right):
            if max_torque is not None:
                motor.set_max_torque(max_torque)
            if torque_limit is not None:
                motor.set_torque_limit(torque_limit)
            motor.set_torque_enable(True)

    def drive(self, v, w):
        """v = linear (+forward), w = angular (+turn left). Both in motor units."""
        left_cmd = v - w
        right_cmd = v + w
        if self.invert_right:
            right_cmd = -right_cmd
        self.left.set_speed(left_cmd)
        self.right.set_speed(right_cmd)

    def stop(self):
        self.left.stop()
        self.right.stop()

    def close(self):
        try:
            self.stop()
        finally:
            self.ser.close()


def keyboard_loop(drive, step=250, max_speed=1023):
    import select
    import termios
    import tty

    fd = sys.stdin.fileno()
    old_attrs = termios.tcgetattr(fd)
    v = 0
    w = 0
    print(
        "w/s = forward/reverse | a/d = turn left/right | "
        "space = stop | [ / ] step | q = quit"
    )
    print(f"step={step}, max={max_speed}")

    def clamp(x):
        return max(-max_speed, min(max_speed, x))

    try:
        tty.setcbreak(fd)
        while True:
            ready, _, _ = select.select([sys.stdin], [], [], 0.05)
            if not ready:
                continue
            c = sys.stdin.read(1)
            if c == "q":
                break
            elif c == "w":
                v = clamp(v + step)
            elif c == "s":
                v = clamp(v - step)
            elif c == "a":
                w = clamp(w + step)
            elif c == "d":
                w = clamp(w - step)
            elif c == " ":
                v = w = 0
            elif c == "[":
                step = max(10, step - 50)
            elif c == "]":
                step = min(1000, step + 50)
            elif c == "\x1b":
                seq = sys.stdin.read(2)
                if seq == "[A":
                    v = clamp(v + step)
                elif seq == "[B":
                    v = clamp(v - step)
                elif seq == "[D":
                    w = clamp(w + step)
                elif seq == "[C":
                    w = clamp(w - step)
                else:
                    continue
            else:
                continue
            drive.drive(v, w)
            print(f"\rv={v:+5d}  w={w:+5d}  step={step:3d}   ", end="", flush=True)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_attrs)
        print()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("port", help="Serial port, e.g. /dev/tty.usbmodemXXXX")
    parser.add_argument("--left-id", type=int, default=LEFT_ID)
    parser.add_argument("--right-id", type=int, default=RIGHT_ID)
    parser.add_argument("--no-invert-right", action="store_true",
                        help="Disable mirror-mount sign flip on the right motor")
    parser.add_argument("--step", type=int, default=250)
    parser.add_argument("--max", type=int, default=1023)
    parser.add_argument("--max-torque", type=int, default=700,
                        help="EEPROM Max Torque (0-1000, persisted)")
    parser.add_argument("--torque-limit", type=int, default=700,
                        help="RAM Torque Limit (0-1000, runtime cap)")
    args = parser.parse_args()

    ser = serial.Serial(args.port, baudrate=BAUDRATE, timeout=0.05)
    drive = DiffDrive(
        ser,
        left_id=args.left_id,
        right_id=args.right_id,
        invert_right=not args.no_invert_right,
    )
    try:
        status = drive.ping()
        for side, ok in status.items():
            print(f"{side} (ID {drive.left.id if side == 'left' else drive.right.id}): "
                  f"{'OK' if ok else 'NO RESPONSE'}")
        if not all(status.values()):
            print("At least one motor did not answer. Check IDs / wiring.", file=sys.stderr)
            sys.exit(1)
        drive.apply_torque(max_torque=args.max_torque, torque_limit=args.torque_limit)
        print(f"Max Torque (EEPROM) = {args.max_torque}, "
              f"Torque Limit (RAM) = {args.torque_limit} (scale: 0-1000)")
        keyboard_loop(drive, step=args.step, max_speed=args.max)
    finally:
        drive.close()


if __name__ == "__main__":
    main()
