"""Scoring logic (pure Python, unit tested in sim/evaluation/tests)."""

from __future__ import annotations

import csv
import json
import sqlite3
from pathlib import Path

_RANK = {"none": 0, "pose": 1, "label": 2}


def load_records(path: str | Path) -> list[dict]:
    """Scan records from the inventory SQLite queue or a CSV export."""
    path = Path(path)
    if path.suffix == ".csv":
        with path.open(newline="") as f:
            return [{**row, "level": int(row["level"]) if row.get("level") else None} for row in csv.DictReader(f)]
    with sqlite3.connect(str(path)) as db:
        return [json.loads(r[0]) for r in db.execute("SELECT record_json FROM scans ORDER BY created_at")]


def best_reads(records: list[dict]) -> dict[str, dict]:
    """One record per decoded code: the best attribution method, latest on ties."""
    best: dict[str, dict] = {}
    for r in records:
        old = best.get(r["code"])
        if old is None or _RANK.get(r["attribution_method"], 0) >= _RANK.get(old["attribution_method"], 0):
            best[r["code"]] = r
    return best


def score(ground_truth: dict, records: list[dict]) -> dict:
    truth: dict[str, str] = ground_truth["items"]  # item code -> slot code
    location_codes = {s["location_code"] for s in ground_truth["slots"]}
    reads = {code: r for code, r in best_reads(records).items() if code not in location_codes}
    true_reads = sorted(c for c in reads if c in truth)
    false_reads = sorted(c for c in reads if c not in truth)
    correct = sorted(c for c in true_reads if reads[c].get("slot_code") == truth[c])
    wrong = [
        {
            "code": c,
            "attributed": reads[c].get("slot_code") or None,
            "truth": truth[c],
            "method": reads[c]["attribution_method"],
        }
        for c in true_reads
        if c not in correct
    ]
    methods: dict[str, int] = {}
    for c in true_reads:
        methods[reads[c]["attribution_method"]] = methods.get(reads[c]["attribution_method"], 0) + 1
    n_truth, n_true = len(truth), len(true_reads)
    return {
        "warehouse_id": ground_truth.get("warehouse_id"),
        "items_on_shelves": n_truth,
        "items_read": n_true,
        "read_recall": n_true / n_truth if n_truth else 0.0,
        "false_reads": len(false_reads),
        "false_read_codes": false_reads,
        "correct_slot": len(correct),
        "correct_slot_rate": len(correct) / n_true if n_true else 0.0,
        "attribution_methods": methods,
        "missed": sorted(set(truth) - set(true_reads)),
        "wrong_slot": wrong,
    }


def format_report(report: dict) -> str:
    r = report
    methods = ", ".join(f"{k}={v}" for k, v in sorted(r["attribution_methods"].items())) or "-"
    lines = [
        f"Mission evaluation: {r['warehouse_id']}",
        f"  read recall        {r['read_recall']:6.1%}  ({r['items_read']}/{r['items_on_shelves']} items)",
        f"  false reads        {r['false_reads']:6d}",
        f"  correct-slot rate  {r['correct_slot_rate']:6.1%}  ({r['correct_slot']}/{r['items_read']} read items)",
        f"  attribution        {methods}",
    ]
    if report["wrong_slot"]:
        lines.append("  wrong slot:")
        lines += [
            f"    {w['code']}: {w['attributed']} (truth {w['truth']}, {w['method']})" for w in report["wrong_slot"][:20]
        ]
    if report["missed"]:
        shown = ", ".join(report["missed"][:15]) + (" ..." if len(report["missed"]) > 15 else "")
        lines.append(f"  missed ({len(report['missed'])}): {shown}")
    return "\n".join(lines)
