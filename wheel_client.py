"""Remote wheel teleop: drive the wheels XoQ server with the same keyboard UI
as wheel_control.py.

The xoq_serial import hook makes `serial.Serial(<64-char-iroh-id>, ...)` open
a remote port, so DiffDrive / WheelMotor / keyboard_loop from wheel_control.py
work unchanged — we just feed them the wheels ID from xoq_config.json instead
of a /dev/... path.

Run:
  python wheel_client.py
  python wheel_client.py --config /path/to/xoq_config.json
  python wheel_client.py --id <64-char-hex>          # bypass config file
"""

import argparse
import json
import sys
from pathlib import Path

try:
    import xoq
except ImportError:
    print(
        "error: `xoq` is required for remote serial ports.\n"
        "  fix: run `uv sync` in this project, then `uv run python wheel_client.py ...`",
        file=sys.stderr,
    )
    sys.exit(2)

from wheel_control import DiffDrive, LEFT_ID, RIGHT_ID, keyboard_loop

DEFAULT_CONFIG = Path(__file__).resolve().parent / "xoq_config.json"

# Default wheels iroh ID — points at the public xlerobot demo base. If you run
# your own xoq_servers.py the generated xoq_config.json takes precedence, and
# --id always wins. iroh identities are persistent (see xoq_servers.py
# --key-root), so this only needs updating if the base is rebuilt.
DEFAULT_WHEELS_ID = "9487f3e866e131f4010cc42c5c554f1dbf14379fbf77cb583fbe472509506403"


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG,
                        help="xoq_config.json from xoq_servers.py")
    parser.add_argument("--id", default=None,
                        help="64-char iroh ID for wheels (overrides --config)")
    parser.add_argument("--left-id", type=int, default=LEFT_ID)
    parser.add_argument("--right-id", type=int, default=RIGHT_ID)
    parser.add_argument("--no-invert-right", action="store_true")
    parser.add_argument("--step", type=int, default=250)
    parser.add_argument("--max", type=int, default=1023)
    parser.add_argument("--max-torque", type=int, default=700)
    parser.add_argument("--torque-limit", type=int, default=700)
    args = parser.parse_args()

    if args.id:
        wheels_id = args.id
        source = "--id"
    elif args.config.exists():
        cfg = json.loads(args.config.read_text())
        try:
            wheels_id = cfg["servers"]["wheels"]["id"]
        except KeyError:
            print(f"error: config has no 'wheels' server entry: {args.config}",
                  file=sys.stderr)
            return 2
        source = str(args.config)
    else:
        wheels_id = DEFAULT_WHEELS_ID
        source = "built-in default"

    if wheels_id is None or len(wheels_id) != 64:
        print(f"error: invalid wheels iroh ID: {wheels_id!r}", file=sys.stderr)
        return 2

    print(f"wheels ID source: {source}")

    print(f"Connecting to wheels server  {wheels_id[:16]}…")
    ser = xoq.serial.Serial(wheels_id, timeout=0.05)
    drive = DiffDrive(
        ser,
        left_id=args.left_id,
        right_id=args.right_id,
        invert_right=not args.no_invert_right,
    )
    try:
        status = drive.ping()
        for side, ok in status.items():
            sid = drive.left.id if side == "left" else drive.right.id
            print(f"{side} (ID {sid}): {'OK' if ok else 'NO RESPONSE'}")
        if not all(status.values()):
            print("At least one wheel motor did not answer. Check IDs / wiring "
                  "/ that the wheels server is bound to the right /dev port.",
                  file=sys.stderr)
            return 1

        drive.apply_torque(max_torque=args.max_torque,
                           torque_limit=args.torque_limit)
        print(f"Max Torque (EEPROM) = {args.max_torque}, "
              f"Torque Limit (RAM) = {args.torque_limit} (scale: 0-1000)")
        keyboard_loop(drive, step=args.step, max_speed=args.max)
    finally:
        drive.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())
