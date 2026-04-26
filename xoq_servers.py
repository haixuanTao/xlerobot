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

DEFAULT_PORTS = {
    "left-arm":  "/dev/tty.usbmodem-ARM-L",
    "right-arm": "/dev/tty.usbmodem-ARM-R",
    "wheels":    "/dev/tty.usbmodem-WHEELS",
}

LABEL_WIDTH = max(len(name) for name in DEFAULT_PORTS) + 1


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
    parser.add_argument("--left-arm-port",  default=DEFAULT_PORTS["left-arm"])
    parser.add_argument("--right-arm-port", default=DEFAULT_PORTS["right-arm"])
    parser.add_argument("--wheels-port",    default=DEFAULT_PORTS["wheels"])
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

    requested = {
        "left-arm":  args.left_arm_port,
        "right-arm": args.right_arm_port,
        "wheels":    args.wheels_port,
    }
    missing = {name: p for name, p in requested.items() if not os.path.exists(p)}
    if missing:
        print("error: the following serial ports do not exist:", file=sys.stderr)
        for name, p in missing.items():
            print(f"  {name}: {p}", file=sys.stderr)
        available = sorted(glob.glob("/dev/tty.usbmodem*") + glob.glob("/dev/ttyUSB*"))
        if available:
            print("\nAvailable usbmodem/ttyUSB ports right now:", file=sys.stderr)
            for p in available:
                print(f"  {p}", file=sys.stderr)
            print("\nRe-run with explicit flags, e.g.:", file=sys.stderr)
            print(f"  python {Path(sys.argv[0]).name} \\\n"
                  f"    --left-arm-port  {available[0] if len(available) > 0 else '<port>'} \\\n"
                  f"    --right-arm-port {available[1] if len(available) > 1 else '<port>'} \\\n"
                  f"    --wheels-port    {available[2] if len(available) > 2 else '<port>'}",
                  file=sys.stderr)
        else:
            print("\nNo /dev/tty.usbmodem* or /dev/ttyUSB* devices detected. "
                  "Plug in the Feetech adapters first.", file=sys.stderr)
        return 2

    binary = build_binary(args.wser_dir)

    plan = [
        ("left-arm",  args.left_arm_port),
        ("right-arm", args.right_arm_port),
        ("wheels",    args.wheels_port),
    ]

    servers: list[Server] = []
    for name, port in plan:
        moq_path = f"anon/xlerobot-{name}" if args.moq_relay else None
        proc = spawn_server(
            name=name,
            port=port,
            baud=args.baud,
            key_dir=args.key_root / name,
            binary=binary,
            wser_dir=args.wser_dir,
            moq_relay=args.moq_relay,
        )
        servers.append(Server(
            name=name,
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
        config = {
            "version": 1,
            "servers": {
                s.name: {
                    "id": s.server_id,
                    "port": s.port,
                    "baud": s.baud,
                    "moq_relay": s.moq_relay,
                    "moq_path": s.moq_path,
                }
                for s in servers
            },
        }
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
            print("\n=== XoQ server IDs ===", flush=True)
            for s in servers:
                print(f"{s.name:<{LABEL_WIDTH}} : {s.server_id}", flush=True)
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
