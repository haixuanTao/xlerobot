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
from dataclasses import dataclass
from pathlib import Path

import serial  # patched by xoq_serial: 64-hex IDs route over iroh

# --- Feetech STS3215 (protocol v1) ---------------------------------------
INST_PING = 0x01
INST_READ = 0x02
INST_WRITE = 0x03
INST_SYNC_WRITE = 0x83

ADDR_TORQUE_ENABLE = 40        # 1 byte
ADDR_GOAL_POSITION = 42        # 2 bytes
ADDR_PRESENT_POSITION = 56     # 2 bytes

DEFAULT_SERVO_IDS = [1, 2, 3, 4, 5, 6]
DEFAULT_BAUD = 1_000_000
DEFAULT_CONFIG = Path(__file__).resolve().parent / "xoq_config.json"


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


class FeetechBus:
    """Minimal STS3215 v1 driver. Works on real or xoq remote serial ports."""

    def __init__(self, port_id, baudrate=DEFAULT_BAUD, timeout=0.05, label=""):
        self.ser = serial.Serial(port_id, baudrate=baudrate, timeout=timeout)
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


@dataclass
class ArmPair:
    name: str
    leader: FeetechBus
    follower: FeetechBus
    ids: list


def teleop_loop(pairs, rate_hz, stop):
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

        loop += 1
        if loop % (int(rate_hz) * 2) == 0:
            dt = time.monotonic() - t0
            print(f"[teleop] tick {loop}  last cycle {dt*1000:.1f}ms", flush=True)

        sleep_for = period - (time.monotonic() - t0)
        if sleep_for > 0:
            stop.wait(sleep_for)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG,
                        help="xoq_config.json written by xoq_servers.py")
    parser.add_argument("--leader-left-port", default=None)
    parser.add_argument("--leader-right-port", default=None)
    parser.add_argument("--side", choices=["both", "left", "right"], default="both")
    parser.add_argument("--rate-hz", type=float, default=100.0)
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--ids", default=",".join(str(x) for x in DEFAULT_SERVO_IDS),
                        help="Servo IDs comma-separated (default 1-6)")
    parser.add_argument("--no-confirm", action="store_true",
                        help="Skip the 'press Enter to start' safety prompt")
    args = parser.parse_args()

    if not args.config.exists():
        print(f"error: config not found: {args.config}\n"
              f"  run xoq_servers.py first to create it.", file=sys.stderr)
        return 2
    cfg = json.loads(args.config.read_text())["servers"]
    for k in ("left-arm", "right-arm"):
        if k not in cfg:
            print(f"error: config missing '{k}' entry", file=sys.stderr)
            return 2

    ids = [int(x) for x in args.ids.split(",") if x.strip()]

    plan = []
    if args.side in ("both", "left"):
        if not args.leader_left_port:
            print("error: --leader-left-port required for left side", file=sys.stderr)
            return 2
        plan.append(("left-arm", args.leader_left_port, cfg["left-arm"]["id"]))
    if args.side in ("both", "right"):
        if not args.leader_right_port:
            print("error: --leader-right-port required for right side", file=sys.stderr)
            return 2
        plan.append(("right-arm", args.leader_right_port, cfg["right-arm"]["id"]))

    pairs = []
    for name, leader_port, follower_id in plan:
        print(f"[{name}] opening leader (local): {leader_port}", flush=True)
        leader = FeetechBus(leader_port, baudrate=args.baud, label=f"{name}-leader")
        print(f"[{name}] opening follower (xoq):  {follower_id[:16]}…", flush=True)
        follower = FeetechBus(follower_id, baudrate=args.baud, label=f"{name}-follower")
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

    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())

    print(f"\nTeleop active @ {args.rate_hz:.0f}Hz. Ctrl-C to stop.\n", flush=True)
    try:
        teleop_loop(pairs, args.rate_hz, stop)
    finally:
        print("\nShutting down: disabling follower torque, closing ports...",
              flush=True)
        for p in pairs:
            try:
                p.follower.set_torque(p.ids, False)
            except Exception as e:
                print(f"[{p.name}] follower torque-off failed: {e}", flush=True)
            p.leader.close()
            p.follower.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())
