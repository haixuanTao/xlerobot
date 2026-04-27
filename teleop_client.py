"""SO-100 teleop client: local leader arms drive remote follower arms via XoQ.

Reads present positions from 1-2 local leader arms (real pyserial) and mirrors
them to the remote follower arms exposed by xoq_servers.py. Both transports
go through the same `serial.Serial` API thanks to the xoq_serial import hook —
a 64-char hex port is routed over iroh, anything else is normal pyserial.

Run:
  python teleop_client.py \
    --leader-left-port  /dev/tty.usbmodem-LEADER-L \
    --leader-right-port /dev/tty.usbmodem-LEADER-R
  # or single-side:
  python teleop_client.py --side left --leader-left-port /dev/tty.usbmodem...
"""

import argparse
import json
import signal
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path

import serial as pyserial  # only for local /dev/tty.* leader arms

try:
    import xoq  # for remote follower arms (xoq.serial.Serial)
except ImportError:
    print(
        "error: `xoq` is required for remote serial ports.\n"
        "  fix: run `uv sync` in this project, then `uv run python teleop_client.py ...`",
        file=sys.stderr,
    )
    sys.exit(2)

# --- Feetech STS3215 (protocol v1) ---------------------------------------
INST_PING = 0x01
INST_READ = 0x02
INST_WRITE = 0x03
INST_SYNC_WRITE = 0x83

ADDR_TORQUE_ENABLE = 40        # 1 byte
ADDR_GOAL_POSITION = 42        # 2 bytes (arms)
ADDR_GOAL_SPEED = 46           # 2 bytes (wheels, signed via bit 15)
ADDR_TORQUE_LIMIT = 48         # 2 bytes RAM, 0-1000
ADDR_PRESENT_POSITION = 56     # 2 bytes

DEFAULT_SERVO_IDS = [1, 2, 3, 4, 5, 6]
DEFAULT_BAUD = 1_000_000
DEFAULT_CONFIG = Path(__file__).resolve().parent / "xoq_config.json"

# Default follower iroh IDs — point at the public xlerobot demo arms.
DEFAULT_FOLLOWER_IDS = {
    "left-arm":  "bbe73cf93555520472aeb48ebe62677099887e9b64ef53590740d0c78e7c5060",
    "right-arm": "02d5f49e541f0777eca479d240babf8d0f9f33b81d49c13d7c3a77a06af2afad",
}
DEFAULT_WHEELS_ID = "bbe73cf93555520472aeb48ebe62677099887e9b64ef53590740d0c78e7c5060"

# Wheels: STS3215 on the same Feetech bus as the left arm (combined Waveshare).
WHEEL_LEFT_ID = 7
WHEEL_RIGHT_ID = 8
WHEEL_SPEED_LIMIT = 1023
WHEEL_INVERT_RIGHT = True
WHEEL_TORQUE_LIMIT = 700


def _checksum(body):
    return (~sum(body)) & 0xFF


def _packet(motor_id, instruction, params=()):
    length = len(params) + 2
    body = [motor_id, length, instruction, *params]
    return bytes([0xFF, 0xFF, *body, _checksum(body)])


def _sync_write_packet(addr, data_len, items):
    """items: list of (id, [byte0, byte1, ...]) all of length data_len."""
    params = [addr, data_len]
    for sid, data in items:
        params.append(sid)
        params.extend(data)
    length = len(params) + 2
    body = [0xFE, length, INST_SYNC_WRITE, *params]
    return bytes([0xFF, 0xFF, *body, _checksum(body)])


def open_local_bus(path, baudrate=DEFAULT_BAUD, timeout=0.05):
    return pyserial.Serial(path, baudrate=baudrate, timeout=timeout)


def open_remote_bus(node_id, timeout=0.05):
    return xoq.serial.Serial(node_id, timeout=timeout)


class FeetechBus:
    """Minimal STS3215 v1 driver over a pre-opened serial port (any transport)."""

    def __init__(self, ser, label=""):
        self.ser = ser
        self.label = label

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass

    def _write_inst(self, motor_id, addr, data):
        self.ser.reset_input_buffer()
        self.ser.write(_packet(motor_id, INST_WRITE, [addr, *data]))
        # Drain status response so the next read isn't polluted.
        self.ser.read(6)

    def ping(self, motor_id):
        self.ser.reset_input_buffer()
        self.ser.write(_packet(motor_id, INST_PING))
        resp = self.ser.read(6)
        return len(resp) == 6 and resp[0] == 0xFF and resp[1] == 0xFF and resp[2] == motor_id

    def set_torque(self, ids, enabled):
        for sid in ids:
            self._write_inst(sid, ADDR_TORQUE_ENABLE, [1 if enabled else 0])
            time.sleep(0.005)

    def read_present_positions(self, ids):
        """Return {id: int16-position}. Missing IDs are absent from the dict."""
        out = {}
        for sid in ids:
            self.ser.reset_input_buffer()
            self.ser.write(_packet(sid, INST_READ, [ADDR_PRESENT_POSITION, 2]))
            resp = self.ser.read(8)
            if (len(resp) == 8 and resp[0] == 0xFF and resp[1] == 0xFF
                    and resp[2] == sid):
                out[sid] = resp[5] | (resp[6] << 8)
        return out

    def sync_write_goal_positions(self, positions):
        items = []
        for sid, pos in positions.items():
            pos = int(pos) & 0xFFFF
            items.append((sid, [pos & 0xFF, (pos >> 8) & 0xFF]))
        if items:
            self.ser.write(_sync_write_packet(ADDR_GOAL_POSITION, 2, items))

    def write_byte_register(self, motor_id, addr, data):
        """Public wrapper around _write_inst for callers that need a single
        register write (e.g. wheel torque-limit setup)."""
        self._write_inst(motor_id, addr, data)


# ---- Wheels --------------------------------------------------------------

@dataclass
class WheelState:
    v: int = 0
    w: int = 0
    step: int = 250
    max_speed: int = WHEEL_SPEED_LIMIT
    lock: "threading.Lock" = field(default_factory=threading.Lock)

    def snapshot(self):
        with self.lock:
            return self.v, self.w, self.step


def _pack_wheel_speed(signed):
    speed = max(-WHEEL_SPEED_LIMIT, min(WHEEL_SPEED_LIMIT, int(signed)))
    raw = (-speed) | 0x8000 if speed < 0 else speed
    return [raw & 0xFF, (raw >> 8) & 0xFF]


def write_wheel_speeds(bus, v, w):
    """Diff drive (v=forward, w=turn-left). Single sync_write to IDs 7+8."""
    left = v - w
    right = v + w
    if WHEEL_INVERT_RIGHT:
        right = -right
    items = [
        (WHEEL_LEFT_ID,  _pack_wheel_speed(left)),
        (WHEEL_RIGHT_ID, _pack_wheel_speed(right)),
    ]
    bus.ser.write(_sync_write_packet(ADDR_GOAL_SPEED, 2, items))


def configure_wheels(bus, torque_limit=WHEEL_TORQUE_LIMIT):
    """Set RAM Torque Limit, zero goal_speed, then enable torque."""
    for sid in (WHEEL_LEFT_ID, WHEEL_RIGHT_ID):
        bus.write_byte_register(sid, ADDR_TORQUE_LIMIT,
                                [torque_limit & 0xFF, (torque_limit >> 8) & 0xFF])
    write_wheel_speeds(bus, 0, 0)
    bus.set_torque((WHEEL_LEFT_ID, WHEEL_RIGHT_ID), True)


def keyboard_thread(state, stop):
    import select
    import termios
    import tty

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        print("\n[wheels] w/s = forward/reverse | a/d = turn left/right | "
              "space = stop | [/] = step | q = quit", flush=True)
        while not stop.is_set():
            ready, _, _ = select.select([sys.stdin], [], [], 0.1)
            if not ready:
                continue
            c = sys.stdin.read(1)
            with state.lock:
                if c == 'q':
                    state.v = state.w = 0
                    stop.set()
                elif c == 'w':
                    state.v = max(-state.max_speed, min(state.max_speed, state.v + state.step))
                elif c == 's':
                    state.v = max(-state.max_speed, min(state.max_speed, state.v - state.step))
                elif c == 'a':
                    state.w = max(-state.max_speed, min(state.max_speed, state.w + state.step))
                elif c == 'd':
                    state.w = max(-state.max_speed, min(state.max_speed, state.w - state.step))
                elif c == ' ':
                    state.v = state.w = 0
                elif c == '[':
                    state.step = max(10, state.step - 50)
                elif c == ']':
                    state.step = min(1000, state.step + 50)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


# ---- Arm teleop ----------------------------------------------------------

@dataclass
class ArmPair:
    name: str
    leader: FeetechBus
    follower: FeetechBus
    ids: list


def teleop_loop(pairs, rate_hz, stop, wheel_bus=None, wheel_state=None):
    period = 1.0 / rate_hz
    last_warn = 0.0
    loop = 0
    while not stop.is_set():
        t0 = time.monotonic()
        for pair in pairs:
            try:
                positions = pair.leader.read_present_positions(pair.ids)
                if len(positions) == len(pair.ids):
                    pair.follower.sync_write_goal_positions(positions)
                elif time.monotonic() - last_warn > 1.0:
                    missing = sorted(set(pair.ids) - set(positions))
                    print(f"[{pair.name}] leader read missing IDs {missing} "
                          f"({len(positions)}/{len(pair.ids)})", flush=True)
                    last_warn = time.monotonic()
            except Exception as e:
                print(f"[{pair.name}] loop error: {e}", flush=True)

        if wheel_bus is not None and wheel_state is not None:
            try:
                v, w, _ = wheel_state.snapshot()
                write_wheel_speeds(wheel_bus, v, w)
            except Exception as e:
                print(f"[wheels] write error: {e}", flush=True)

        loop += 1
        if loop % (int(rate_hz) * 2) == 0:
            dt = time.monotonic() - t0
            extra = ""
            if wheel_state is not None:
                v, w, step = wheel_state.snapshot()
                extra = f"  wheels v={v:+5d} w={w:+5d} step={step}"
            print(f"[teleop] tick {loop}  last cycle {dt*1000:.1f}ms{extra}",
                  flush=True)

        sleep_for = period - (time.monotonic() - t0)
        if sleep_for > 0:
            stop.wait(sleep_for)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG,
                        help="xoq_config.json written by xoq_servers.py")
    parser.add_argument("--leader-left-port", default=None)
    parser.add_argument("--leader-right-port", default=None)
    parser.add_argument("--left-arm-id", default=None,
                        help="Override follower iroh ID for left arm")
    parser.add_argument("--right-arm-id", default=None,
                        help="Override follower iroh ID for right arm")
    parser.add_argument("--side", choices=["both", "left", "right"], default="both")
    parser.add_argument("--rate-hz", type=float, default=100.0)
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--ids", default=",".join(str(x) for x in DEFAULT_SERVO_IDS),
                        help="Servo IDs comma-separated (default 1-6)")
    parser.add_argument("--no-confirm", action="store_true",
                        help="Skip the 'press Enter to start' safety prompt")
    parser.add_argument("--no-wheels", action="store_true",
                        help="Skip wheel keyboard control even if left-arm bus is open")
    parser.add_argument("--wheels-id", default=None,
                        help="Override iroh ID for wheels bus. If equal to the "
                             "left-arm follower ID (combined Waveshare), the "
                             "wheel writes share that connection.")
    args = parser.parse_args()

    cfg = {}
    if args.config.exists():
        try:
            cfg = json.loads(args.config.read_text()).get("servers", {})
        except (json.JSONDecodeError, OSError) as e:
            print(f"warning: failed to read {args.config} ({e}); falling back to "
                  f"built-in defaults", file=sys.stderr)

    def resolve_follower_id(slot, override):
        if override:
            return override, "--*-arm-id"
        if slot in cfg and cfg[slot].get("id"):
            return cfg[slot]["id"], str(args.config)
        return DEFAULT_FOLLOWER_IDS[slot], "built-in default"

    ids = [int(x) for x in args.ids.split(",") if x.strip()]

    # With --side both (default), auto-narrow to whichever side(s) actually
    # have a leader port. Only error if neither side is available.
    sides = []
    if args.side == "both":
        if args.leader_left_port:
            sides.append("left")
        if args.leader_right_port:
            sides.append("right")
        if not sides:
            print("error: provide --leader-left-port and/or --leader-right-port",
                  file=sys.stderr)
            return 2
        if len(sides) == 1:
            print(f"[info] only {sides[0]} leader port given — running single-arm",
                  flush=True)
    else:
        sides = [args.side]

    plan = []
    if "left" in sides:
        if not args.leader_left_port:
            print("error: --leader-left-port required for left side", file=sys.stderr)
            return 2
        fid, src = resolve_follower_id("left-arm", args.left_arm_id)
        print(f"[left-arm]  follower ID source: {src}")
        plan.append(("left-arm", args.leader_left_port, fid))
    if "right" in sides:
        if not args.leader_right_port:
            print("error: --leader-right-port required for right side", file=sys.stderr)
            return 2
        fid, src = resolve_follower_id("right-arm", args.right_arm_id)
        print(f"[right-arm] follower ID source: {src}")
        plan.append(("right-arm", args.leader_right_port, fid))

    pairs = []
    for name, leader_port, follower_id in plan:
        print(f"[{name}] opening leader (pyserial): {leader_port}", flush=True)
        leader = FeetechBus(
            open_local_bus(leader_port, baudrate=args.baud),
            label=f"{name}-leader",
        )
        print(f"[{name}] opening follower (xoq):    {follower_id[:16]}…", flush=True)
        follower = FeetechBus(
            open_remote_bus(follower_id),
            label=f"{name}-follower",
        )
        pairs.append(ArmPair(name=name, leader=leader, follower=follower, ids=ids))

    # Pings — useful early signal that buses are alive and IDs match.
    for p in pairs:
        ok_lead = [sid for sid in p.ids if p.leader.ping(sid)]
        ok_foll = [sid for sid in p.ids if p.follower.ping(sid)]
        print(f"[{p.name}] ping leader   {ok_lead}", flush=True)
        print(f"[{p.name}] ping follower {ok_foll}", flush=True)

    # Safety sequence: drop torque on both, snap follower goal to leader's
    # current pose, then re-enable follower torque so the first frame doesn't
    # cause a violent jerk to a stale goal position.
    print("Disabling all torque...", flush=True)
    for p in pairs:
        p.follower.set_torque(p.ids, False)
        p.leader.set_torque(p.ids, False)

    print("Seeding follower goal from leader pose...", flush=True)
    for p in pairs:
        seed = p.leader.read_present_positions(p.ids)
        if seed:
            p.follower.sync_write_goal_positions(seed)
            time.sleep(0.05)

    if not args.no_confirm:
        try:
            input("\n>>> Press Enter to enable follower torque and start teleop "
                  "(Ctrl-C to abort) ")
        except (EOFError, KeyboardInterrupt):
            print("\naborted before torque-on", flush=True)
            return 0

    print("Enabling follower torque...", flush=True)
    for p in pairs:
        p.follower.set_torque(p.ids, True)

    # ---- Wheels: only if left-arm is in scope and not opted out --------------
    wheel_bus = None
    wheel_state = None
    own_wheel_bus = False  # True if we opened a separate xoq connection
    left_pair = next((p for p in pairs if p.name == "left-arm"), None)

    if left_pair is not None and not args.no_wheels:
        left_arm_id = next(fid for n, _, fid in plan if n == "left-arm")
        wheels_id = (
            args.wheels_id
            or (cfg.get("wheels", {}).get("id"))
            or DEFAULT_WHEELS_ID
        )
        if wheels_id == left_arm_id:
            print(f"[wheels] sharing left-arm bus  (combined Waveshare)", flush=True)
            wheel_bus = left_pair.follower
        else:
            print(f"[wheels] opening separate xoq bus: {wheels_id[:16]}…", flush=True)
            wheel_bus = FeetechBus(open_remote_bus(wheels_id), label="wheels")
            own_wheel_bus = True

        ok = [sid for sid in (WHEEL_LEFT_ID, WHEEL_RIGHT_ID) if wheel_bus.ping(sid)]
        if len(ok) == 2:
            print(f"[wheels] ping IDs {ok} OK", flush=True)
            configure_wheels(wheel_bus)
            wheel_state = WheelState()
        else:
            print(f"[wheels] ping found only {ok} — disabling wheel control",
                  flush=True)
            if own_wheel_bus:
                wheel_bus.close()
            wheel_bus = None
            wheel_state = None
            own_wheel_bus = False

    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())

    if wheel_state is not None:
        kb_thread = threading.Thread(
            target=keyboard_thread, args=(wheel_state, stop), daemon=True
        )
        kb_thread.start()

    print(f"\nTeleop active @ {args.rate_hz:.0f}Hz. Ctrl-C to stop.\n", flush=True)
    try:
        teleop_loop(pairs, args.rate_hz, stop,
                    wheel_bus=wheel_bus, wheel_state=wheel_state)
    finally:
        print("\nShutting down: stopping wheels, disabling follower torque, "
              "closing ports...", flush=True)
        if wheel_bus is not None:
            try:
                write_wheel_speeds(wheel_bus, 0, 0)
                wheel_bus.set_torque((WHEEL_LEFT_ID, WHEEL_RIGHT_ID), False)
            except Exception as e:
                print(f"[wheels] stop/torque-off failed: {e}", flush=True)
        for p in pairs:
            try:
                p.follower.set_torque(p.ids, False)
            except Exception as e:
                print(f"[{p.name}] follower torque-off failed: {e}", flush=True)
            p.leader.close()
            p.follower.close()
        if own_wheel_bus and wheel_bus is not None:
            wheel_bus.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())
