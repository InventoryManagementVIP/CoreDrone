import csv
import json
import sqlite3
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from evaluation.score import best_reads, format_report, load_records, score

GT = {
    "warehouse_id": "test",
    "slots": [
        {"slot_code": "S1", "location_code": "tag36h11:0"},
        {"slot_code": "S2", "location_code": "tag36h11:1"},
        {"slot_code": "S3", "location_code": "tag36h11:2"},
    ],
    "items": {"SKU-1": "S1", "SKU-2": "S2", "SKU-3": "S3"},
}


def rec(code, slot, method="label"):
    return {"code": code, "slot_code": slot, "attribution_method": method}


def test_perfect_run():
    r = score(GT, [rec("SKU-1", "S1"), rec("SKU-2", "S2"), rec("SKU-3", "S3")])
    assert r["read_recall"] == 1.0 and r["correct_slot_rate"] == 1.0 and r["false_reads"] == 0


def test_metrics():
    records = [
        rec("SKU-1", "S1"),
        rec("SKU-2", "S3"),  # wrong slot
        rec("SKU-9", "S2"),  # not on the shelves
        rec("tag36h11:0", None, "none"),  # location tags are not items
    ]
    r = score(GT, records)
    assert r["items_read"] == 2 and r["read_recall"] == pytest.approx(2 / 3)
    assert r["false_reads"] == 1 and r["false_read_codes"] == ["SKU-9"]
    assert r["correct_slot_rate"] == pytest.approx(0.5)
    assert r["wrong_slot"] == [{"code": "SKU-2", "attributed": "S3", "truth": "S2", "method": "label"}]
    assert r["missed"] == ["SKU-3"]
    assert "read recall" in format_report(r)


def test_best_read_prefers_label_attribution():
    best = best_reads([rec("SKU-1", "S2", "pose"), rec("SKU-1", "S1", "label"), rec("SKU-1", None, "none")])
    assert best["SKU-1"]["slot_code"] == "S1"


def test_empty_run():
    r = score(GT, [])
    assert r["read_recall"] == 0.0 and r["correct_slot_rate"] == 0.0 and r["missed"] == ["SKU-1", "SKU-2", "SKU-3"]


def test_load_from_sqlite_and_csv(tmp_path):
    db = tmp_path / "inventory.sqlite"
    with sqlite3.connect(db) as c:
        c.execute("CREATE TABLE scans (scan_id TEXT, created_at TEXT, record_json TEXT)")
        c.execute("INSERT INTO scans VALUES ('a', '1', ?)", (json.dumps({**rec("SKU-1", "S1"), "level": 0}),))
    assert load_records(db)[0]["code"] == "SKU-1"
    path = tmp_path / "scans.csv"
    with path.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["code", "slot_code", "attribution_method", "level"])
        w.writeheader()
        w.writerow({**rec("SKU-2", "S2"), "level": "1"})
    assert load_records(path) == [{**rec("SKU-2", "S2"), "level": 1}]
