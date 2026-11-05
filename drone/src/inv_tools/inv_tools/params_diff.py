"""Compare a flight controller's live parameters with the committed .parm files.

  ros2 run inv_tools params_diff --connection udpin:0.0.0.0:14551 --platform real
  ros2 run inv_tools params_diff --connection tcp:127.0.0.1:5762 --platform sim --check-names
  ros2 run inv_tools params_diff --offline a.parm b.parm

--platform picks the layering: common.parm + real.parm (real) or common.parm + sim.parm (sim).
--check-names only fails on parameter names the firmware does not know (CI uses this against
SITL to catch parameters renamed by an ArduPilot bump). Exit code 1 on any difference.
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path

from inv_tools.parm_files import diff, load_layers, parse_parm_text, same_value


def _root() -> Path:
    env = os.environ.get("INV_ROOT")
    if env:
        return Path(env)
    for parent in Path(__file__).resolve().parents:
        if (parent / "real" / "flight_controller" / "params").is_dir():
            return parent
    raise SystemExit("cannot find the repo root; set INV_ROOT")


def layer_files(platform: str) -> list[Path]:
    root = _root()
    common = root / "real" / "flight_controller" / "params" / "common.parm"
    if platform == "sim":
        return [common, root / "sim" / "params" / "sim.parm"]
    return [common, root / "real" / "flight_controller" / "params" / "real.parm"]


def fetch_params(connection: str, timeout_s: float = 60.0, wanted: tuple[str, ...] = ()) -> dict[str, float]:
    from pymavlink import mavutil

    master = mavutil.mavlink_connection(connection, source_system=246)
    if master.wait_heartbeat(timeout=timeout_s) is None:
        raise SystemExit(f"no heartbeat on {connection}")
    master.param_fetch_all()
    params: dict[str, float] = {}
    expected = None
    deadline = time.monotonic() + timeout_s
    last_new = time.monotonic()
    while time.monotonic() < deadline:
        msg = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=1.0)
        if msg is None:
            if expected and len(params) >= expected:
                break
            if time.monotonic() - last_new > 5.0:
                master.param_fetch_all()  # lost packets: ask again
                last_new = time.monotonic()
            continue
        name = _param_name(msg)
        if name not in params:
            last_new = time.monotonic()
        params[name] = float(msg.param_value)
        expected = msg.param_count
        if expected and len(params) >= expected:
            break
    # Some groups register after the FC has counted its parameters (AP_DDS appears only after
    # "ArduPilot Ready"), so a bulk fetch can miss them: ask for each missing name directly.
    for name in (n for n in wanted if n not in params):
        for _attempt in range(3):
            master.param_fetch_one(name)
            end = time.monotonic() + 1.5
            while time.monotonic() < end and name not in params:
                msg = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
                if msg is not None:
                    params[_param_name(msg)] = float(msg.param_value)
            if name in params:
                break
    return params


def _param_name(msg) -> str:
    raw = msg.param_id
    return (raw if isinstance(raw, str) else raw.decode()).rstrip("\x00")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--connection", help="pymavlink connection string to the FC")
    parser.add_argument("--platform", choices=("real", "sim"), default="real")
    parser.add_argument("--files", nargs="*", help="explicit .parm layering (overrides --platform)")
    parser.add_argument("--check-names", action="store_true", help="only report unknown parameter names")
    parser.add_argument("--offline", nargs=2, metavar=("A", "B"), help="diff two .parm files")
    argv = sys.argv[1:] if argv is None else argv
    if "--ros-args" in argv:
        argv = argv[: argv.index("--ros-args")]
    args = parser.parse_args(argv)

    if args.offline:
        a, b = (parse_parm_text(Path(p).read_text(), p) for p in args.offline)
        changed = 0
        for name in sorted(set(a) | set(b)):
            if name not in a or name not in b or not same_value(a[name], b[name]):
                changed += 1
                print(f"{name:20s} {a.get(name, '-')!s:>12} -> {b.get(name, '-')!s:>12}")
        return 1 if changed else 0

    if not args.connection:
        parser.error("--connection is required unless --offline is used")
    files = [Path(f) for f in args.files] if args.files else layer_files(args.platform)
    expected = load_layers(files)
    actual = fetch_params(args.connection, wanted=tuple(expected))
    mismatches, unknown = diff(expected, actual)
    print(f"checked {len(expected)} parameters from {', '.join(f.name for f in files)} against {len(actual)} on the FC")
    for name, value, source in unknown:
        print(f"UNKNOWN  {name:20s} {value:>12g}  ({source}): the firmware has no such parameter")
    if not args.check_names:
        for name, want, have, source in mismatches:
            print(f"DIFFERS  {name:20s} file={want:<12g} fc={have:<12g} ({source})")
    failed = bool(unknown) or (bool(mismatches) and not args.check_names)
    print("FAIL" if failed else "OK")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
