"""Record a flight to MCAP with the canonical topics (`ros2 run inv_tools record --profile sim`).

Bags are never committed: copy them to the team's dataset storage and list them in
docs/datasets.md. Topic profiles live in config/bag_topics.yaml.
"""

from __future__ import annotations

import argparse
import datetime
import os
import sys
from pathlib import Path

import yaml


def load_profiles() -> dict[str, list[str]]:
    from ament_index_python.packages import get_package_share_directory

    path = Path(get_package_share_directory("inv_tools")) / "config" / "bag_topics.yaml"
    return yaml.safe_load(path.read_text())["profiles"]


def build_command(profile: list[str], output: str, max_size_mb: int) -> list[str]:
    cmd = ["ros2", "bag", "record", "--storage", "mcap", "--output", output]
    if max_size_mb > 0:
        cmd += ["--max-bag-size", str(max_size_mb * 1024 * 1024)]
    return cmd + profile


def main(argv: list[str] | None = None) -> int:
    profiles = load_profiles()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", choices=sorted(profiles), default="minimal")
    parser.add_argument("--output", default="", help="bag directory (default: bags/<profile>_<timestamp>)")
    parser.add_argument("--max-size-mb", type=int, default=2048, help="split size; 0 = no split")
    args = parser.parse_args(argv)
    output = args.output or f"bags/{args.profile}_{datetime.datetime.now():%Y%m%d_%H%M%S}"
    cmd = build_command(profiles[args.profile], output, args.max_size_mb)
    print(" ".join(cmd), flush=True)
    os.execvp(cmd[0], cmd)
    return 0  # unreachable


if __name__ == "__main__":
    sys.exit(main())
