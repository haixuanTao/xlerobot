"""Launch 3 xoq serial-servers (left arm, right arm, wheels) as one process group.

Each Feetech bus gets its own iroh node ID, persisted under --key-root so the
3 IDs stay stable across restarts. Clients hardcode those IDs once.

Run: python xoq_servers.py [--left-arm-port ...] [--right-arm-port ...] [--wheels-port ...]
"""

import argparse
import glob
import json
import os
import re
import shutil
import signal
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path

DEFAULT_WSER_DIR = Path("/Users/xaviertao/Documents/work/wser")
DEFAULT_KEY_ROOT = Path.home() / ".xoq" / "xlerobot"
DEFAULT_BAUD = 1_000_000
DEFAULT_CONFIG_OUT = Path(__file__).resolve().parent / "xoq_config.json"

ID_RE = re.compile(r"\bID:\s*([0-9a-f]{64})\b")

SLOTS = ("left-arm", "right-arm", "wheels")
LABEL_WIDTH = max(len(name) for name in SLOTS) + 1

# Feetech protocol bits — used by the autodetect probe.
WHEEL_IDS = (7, 8)              # both must respond → port is the wheels bus
ARM_PROBE_ID = 1                # SO-100 always has ID 1 → port is an arm
PROBE_TIMEOUT_S = 0.05

try:
    import serial as _pyserial
except ImportError:
    _pyserial = None


def _ping(ser, motor_id, timeout=PROBE_TIMEOUT_S):
    """Send a Feetech PING; return True if the servo responds."""
    body = [motor_id, 0x02, 0x01]  # id, length, INST_PING
    chk = (~sum(body)) & 0xFF
    ser.reset_input_buffer()
    ser.write(bytes([0xFF, 0xFF, *body, chk]))
    ser.flush()
    deadline = time.monotonic() + timeout
    buf = bytearray()
    while time.monotonic() < deadline and len(buf) < 6:
        chunk = ser.read(6 - len(buf))
        if chunk:
            buf.extend(chunk)
    return (len(buf) >= 6 and buf[0] == 0xFF and buf[1] == 0xFF
            and buf[2] == motor_id)


def _probe(path, baud):
    """Return the set of probe IDs that responded on this port (empty on error)."""
    try:
        ser = _pyserial.Serial(path, baudrate=baud, timeout=PROBE_TIMEOUT_S)
    except Exception as e:
        print(f"  {path}: open failed ({e})", flush=True)
        return set()
    try:
        return {sid for sid in (ARM_PROBE_ID, *WHEEL_IDS) if _ping(ser, sid)}
    finally:
        ser.close()


def autodetect(missing_slots, exclude_paths, baud):
    """Probe candidate ports and assign them to the slots in `missing_slots`.

    Returns (assignments_dict, error_message). On failure assignments is None.
    """
    if _pyserial is None:
        return None, "pyserial not installed — cannot autodetect; pass --*-port flags"

    candidates = sorted(set(glob.glob("/dev/tty.usbmodem*") + glob.glob("/dev/ttyUSB*"))
                        - set(exclude_paths))
    if not candidates:
        return None, "no /dev/tty.usbmodem* or /dev/ttyUSB* candidates to probe"

    print(f"[autodetect] probing {len(candidates)} port(s) at {baud} baud "
          f"(IDs {ARM_PROBE_ID}, {WHEEL_IDS[0]}, {WHEEL_IDS[1]})...", flush=True)

    by_path = {}
    for path in candidates:
        ids = _probe(path, baud)
        has_wheels = set(WHEEL_IDS).issubset(ids)
        has_arm = ARM_PROBE_ID in ids
        if has_arm and has_wheels:
            kind = "arm+wheels (combined)"
        elif has_wheels:
            kind = "wheels"
        elif has_arm:
            kind = "arm"
        else:
            kind = "unknown"
        print(f"  {path}: responding={sorted(ids) or '∅'} → {kind}", flush=True)
        by_path[path] = ids

    combined_paths = sorted(p for p, ids in by_path.items()
                            if ARM_PROBE_ID in ids and set(WHEEL_IDS).issubset(ids))
    wheels_only_paths = [p for p, ids in by_path.items()
                         if set(WHEEL_IDS).issubset(ids) and ARM_PROBE_ID not in ids]
    arm_only_paths = sorted(p for p, ids in by_path.items()
                            if ARM_PROBE_ID in ids
                            and not set(WHEEL_IDS).issubset(ids))

    assignments = {}

    if combined_paths:
        # Combined Waveshare: arm + wheels share a single bus. The combined port
        # takes the left-arm slot and wheels.id will alias to it in the config.
        if len(combined_paths) > 1:
            return None, f"ambiguous: multiple arm+wheels buses {combined_paths}"
        combined = combined_paths[0]
        if "left-arm" in missing_slots:
            assignments["left-arm"] = combined
        if "wheels" in missing_slots:
            assignments["wheels"] = combined  # alias: same bus as left-arm
        if "right-arm" in missing_slots:
            if not arm_only_paths:
                return None, ("combined bus assigned to left-arm, but no second "
                              "arm-only bus found for right-arm")
            assignments["right-arm"] = arm_only_paths[0]
        return assignments, None

    # Legacy 3-bus layout: wheels on its own, plus 2 arm-only buses.
    if "wheels" in missing_slots:
        if len(wheels_only_paths) == 0:
            return None, ("no wheels bus detected (no port responded to both "
                          f"ID {WHEEL_IDS[0]} and ID {WHEEL_IDS[1]})")
        if len(wheels_only_paths) > 1:
            return None, f"ambiguous: multiple wheels-only buses {wheels_only_paths}"
        assignments["wheels"] = wheels_only_paths[0]

    arm_slots_needed = [s for s in ("left-arm", "right-arm") if s in missing_slots]
    if arm_slots_needed:
        if len(arm_only_paths) < len(arm_slots_needed):
            return None, (f"need {len(arm_slots_needed)} arm bus(es), "
                          f"found {len(arm_only_paths)}: {arm_only_paths}")
        for slot, path in zip(arm_slots_needed, arm_only_paths):
            assignments[slot] = path

    return assignments, None


@dataclass
class Server:
    name: str
    port: str
    baud: int
    moq_relay: str | None
    moq_path: str | None
    proc: subprocess.Popen
    server_id: str | None = None


CARGO_FEATURES = "iroh serial"


def build_binary(wser_dir: Path) -> Path | None:
    """Build serial-server in release mode. Returns path to binary or None on failure."""
    print(f"[build] cargo build --release --bin serial-server  (in {wser_dir})", flush=True)
    result = subprocess.run(
        ["cargo", "build", "--release", "--bin", "serial-server",
         "--features", CARGO_FEATURES],
        cwd=wser_dir,
    )
    if result.returncode != 0:
        print("[build] FAILED — will fall back to `cargo run` per server", flush=True)
        return None
    binary = wser_dir / "target" / "release" / "serial-server"
    if not binary.exists():
        print(f"[build] binary not found at {binary} — falling back to `cargo run`", flush=True)
        return None
    return binary


def spawn_server(
    name: str,
    port: str,
    baud: int,
    key_dir: Path,
    binary: Path | None,
    wser_dir: Path,
    moq_relay: str | None,
) -> subprocess.Popen:
    key_dir.mkdir(parents=True, exist_ok=True)

    if binary is not None:
        cmd = [str(binary), port, str(baud), "--key-dir", str(key_dir)]
    else:
        cmd = [
            "cargo", "run", "--release", "--bin", "serial-server",
            "--features", CARGO_FEATURES, "--",
            port, str(baud), "--key-dir", str(key_dir),
        ]

    if moq_relay:
        cmd += ["--moq-relay", moq_relay, "--moq-path", f"anon/xlerobot-{name}"]

    env = os.environ.copy()
    env.setdefault("RUST_LOG", "xoq=info,warn")

    return subprocess.Popen(
        cmd,
        cwd=wser_dir,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        env=env,
        bufsize=1,
        text=True,
    )


def stream_output(server: Server) -> None:
    """Tag each line with the server name and capture the printed Server ID."""
    label = f"[{server.name:<{LABEL_WIDTH}}]"
    assert server.proc.stdout is not None
    for line in server.proc.stdout:
        line = line.rstrip()
        print(f"{label} {line}", flush=True)
        if server.server_id is None:
            m = ID_RE.search(line)
            if m:
                server.server_id = m.group(1)


def shutdown(servers: list[Server]) -> None:
    print("\n[shutdown] terminating servers...", flush=True)
    for s in servers:
        if s.proc.poll() is None:
            s.proc.terminate()
    deadline = time.monotonic() + 5.0
    for s in servers:
        remaining = max(0.0, deadline - time.monotonic())
        try:
            s.proc.wait(timeout=remaining)
        except subprocess.TimeoutExpired:
            print(f"[shutdown] {s.name} did not exit in time, sending SIGKILL", flush=True)
            s.proc.kill()
            s.proc.wait()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--left-arm-port",  default=None,
                        help="Skip autodetect; pin left arm to this /dev/tty path")
    parser.add_argument("--right-arm-port", default=None,
                        help="Skip autodetect; pin right arm to this /dev/tty path")
    parser.add_argument("--wheels-port",    default=None,
                        help=f"Skip autodetect; pin wheels (IDs {WHEEL_IDS[0]}+{WHEEL_IDS[1]}) "
                             f"to this /dev/tty path")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--key-root", type=Path, default=DEFAULT_KEY_ROOT)
    parser.add_argument("--wser-dir", type=Path, default=DEFAULT_WSER_DIR)
    parser.add_argument("--moq-relay", default=None,
                        help="Optional MoQ relay URL applied to all 3 servers")
    parser.add_argument("--config-out", type=Path, default=DEFAULT_CONFIG_OUT,
                        help="JSON file to write once all 3 server IDs are known")
    args = parser.parse_args()

    if not args.wser_dir.is_dir():
        print(f"error: --wser-dir does not exist: {args.wser_dir}", file=sys.stderr)
        return 2
    if shutil.which("cargo") is None:
        print("error: `cargo` not on PATH", file=sys.stderr)
        return 2

    pinned = {
        "left-arm":  args.left_arm_port,
        "right-arm": args.right_arm_port,
        "wheels":    args.wheels_port,
    }
    missing_slots = [s for s, p in pinned.items() if p is None]
    if missing_slots:
        assignments, err = autodetect(
            missing_slots=missing_slots,
            exclude_paths=[p for p in pinned.values() if p],
            baud=args.baud,
        )
        if err:
            print(f"error: autodetect failed: {err}", file=sys.stderr)
            available = sorted(glob.glob("/dev/tty.usbmodem*") + glob.glob("/dev/ttyUSB*"))
            if available:
                print("\nAvailable ports right now:", file=sys.stderr)
                for p in available:
                    print(f"  {p}", file=sys.stderr)
                print("\nPin them manually with --left-arm-port / --right-arm-port / "
                      "--wheels-port and re-run.", file=sys.stderr)
            return 2
        for slot, path in assignments.items():
            pinned[slot] = path
        print("[autodetect] assigned:", flush=True)
        for slot in SLOTS:
            print(f"  {slot:<{LABEL_WIDTH}} {pinned[slot]}"
                  f"{'  (pinned)' if slot not in assignments else ''}", flush=True)

    not_present = {s: p for s, p in pinned.items() if not os.path.exists(p)}
    if not_present:
        print("error: the following serial ports do not exist:", file=sys.stderr)
        for s, p in not_present.items():
            print(f"  {s}: {p}", file=sys.stderr)
        return 2

    binary = build_binary(args.wser_dir)

    # Group slots by port: when arm + wheels share one Waveshare we still want
    # only ONE serial-server holding that /dev/tty.* (since OS exclusivity).
    # The owner slot drives the iroh identity; aliased slots reuse its ID.
    SLOT_PRIORITY = ("left-arm", "right-arm", "wheels")  # owner > alias
    by_port: dict[str, str] = {}                         # port -> owner slot
    aliases: dict[str, str] = {}                         # alias slot -> owner slot
    for slot in SLOT_PRIORITY:
        port = pinned[slot]
        if port in by_port:
            aliases[slot] = by_port[port]
        else:
            by_port[port] = slot

    if aliases:
        print("[layout] combined bus detected:", flush=True)
        for alias, owner in aliases.items():
            print(f"  {alias:<{LABEL_WIDTH}} aliased to '{owner}' (same bus: {pinned[alias]})",
                  flush=True)

    servers: list[Server] = []
    for port, owner_slot in by_port.items():
        moq_path = f"anon/xlerobot-{owner_slot}" if args.moq_relay else None
        proc = spawn_server(
            name=owner_slot,
            port=port,
            baud=args.baud,
            key_dir=args.key_root / owner_slot,
            binary=binary,
            wser_dir=args.wser_dir,
            moq_relay=args.moq_relay,
        )
        servers.append(Server(
            name=owner_slot,
            port=port,
            baud=args.baud,
            moq_relay=args.moq_relay,
            moq_path=moq_path,
            proc=proc,
        ))

    threads = []
    for s in servers:
        t = threading.Thread(target=stream_output, args=(s,), daemon=True)
        t.start()
        threads.append(t)

    summary_printed = False

    def write_config():
        owners = {s.name: s for s in servers}
        entries = {}
        for slot in SLOTS:
            owner_slot = aliases.get(slot, slot)
            owner = owners[owner_slot]
            entry = {
                "id": owner.server_id,
                "port": pinned[slot],
                "baud": owner.baud,
                "moq_relay": owner.moq_relay,
                "moq_path": owner.moq_path,
            }
            if slot in aliases:
                entry["alias_of"] = owner_slot
            entries[slot] = entry
        config = {"version": 1, "servers": entries}
        args.config_out.parent.mkdir(parents=True, exist_ok=True)
        tmp = args.config_out.with_suffix(args.config_out.suffix + ".tmp")
        tmp.write_text(json.dumps(config, indent=2) + "\n")
        tmp.replace(args.config_out)

    def maybe_print_summary():
        nonlocal summary_printed
        if summary_printed:
            return
        if all(s.server_id for s in servers):
            write_config()
            owners = {s.name: s for s in servers}
            print("\n=== XoQ server IDs ===", flush=True)
            for slot in SLOTS:
                owner = owners[aliases.get(slot, slot)]
                tag = f" (→ {aliases[slot]})" if slot in aliases else ""
                print(f"{slot:<{LABEL_WIDTH}} : {owner.server_id}{tag}", flush=True)
            print(f"\nconfig written: {args.config_out}", flush=True)
            print("======================\n", flush=True)
            summary_printed = True

    stop = threading.Event()

    def on_signal(signum, _frame):
        stop.set()

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    try:
        while not stop.is_set():
            maybe_print_summary()
            for s in servers:
                rc = s.proc.poll()
                if rc is not None:
                    print(f"[{s.name}] exited with code {rc}", flush=True)
                    stop.set()
                    break
            time.sleep(0.2)
    finally:
        shutdown(servers)
        for t in threads:
            t.join(timeout=1.0)

    return 0 if all(s.proc.returncode == 0 for s in servers) else 1


if __name__ == "__main__":
    sys.exit(main())
