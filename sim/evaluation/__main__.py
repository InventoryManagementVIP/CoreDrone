"""python3 -m evaluation --run-dir runs/latest --world sim_small [--out report.json]

Reads <run-dir>/inventory.sqlite (or --records <file.csv|.sqlite>) and
sim/worlds/generated/<world>/ground_truth.json, prints the report and writes it as JSON.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from evaluation.score import format_report, load_records, score

SIM = Path(__file__).resolve().parents[1]


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--run-dir", type=Path, default=SIM.parent / "runs" / "latest")
    parser.add_argument("--world", default="sim_small")
    parser.add_argument("--records", type=Path, help="CSV export or SQLite queue (default: <run-dir>/inventory.sqlite)")
    parser.add_argument("--ground-truth", type=Path, help="default: sim/worlds/generated/<world>/ground_truth.json")
    parser.add_argument("--out", type=Path, help="report JSON (default: <run-dir>/evaluation.json)")
    args = parser.parse_args(argv)

    records_path = args.records or args.run_dir / "inventory.sqlite"
    gt_path = args.ground_truth or SIM / "worlds" / "generated" / args.world / "ground_truth.json"
    for p in (records_path, gt_path):
        if not p.exists():
            print(f"error: {p} not found", file=sys.stderr)
            return 1
    report = score(json.loads(gt_path.read_text()), load_records(records_path))
    out = args.out or args.run_dir / "evaluation.json"
    out.write_text(json.dumps(report, indent=2) + "\n")
    print(format_report(report))
    print(f"report: {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
