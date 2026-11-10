import csv
import datetime as dt
import json
import pathlib
import sqlite3

import pytest
import yaml

from inv_inventory.queue import ScanQueue
from inv_inventory.records import make_scan_record
from inv_uplink.backends import BackendError, HttpBackend, MockBackend
from inv_uplink.export import COLUMNS, contract_errors, export_csv, resolve_export_path, safe_name_part
from inv_uplink.node import DEFAULTS, UplinkNode

ROOT = pathlib.Path(__file__).resolve().parents[4]


def record(code="SKU-1", **kw):
    base = {
        "mission_id": "m1",
        "drone_id": "d1",
        "warehouse_id": "sim_small",
        "timestamp": dt.datetime.now(dt.timezone.utc),
        "code": code,
        "symbology": "qr",
        "slot_code": "A1-R-L0-B0",
        "rack_id": "A1-R",
        "level": 0,
        "position_map": (1.0, 2.0, 0.5),
        "confidence": 0.9,
        "attribution_method": "label",
    }
    base.update(kw)
    return make_scan_record(**base)


def test_valid_records_pass_the_contract():
    assert contract_errors(record()) == []
    assert (
        contract_errors(record(slot_code=None, rack_id=None, level=None, position_map=None, attribution_method="none"))
        == []
    )


def test_contract_catches_bad_records():
    bad = record()
    bad["symbology"] = "pdf417"
    bad["extra"] = 1
    del bad["drone_id"]
    errors = " ".join(contract_errors(bad))
    assert "pdf417" in errors and "extra" in errors and "drone_id" in errors


def test_contract_example_file_is_valid():
    example = json.loads((ROOT / "contracts" / "examples" / "scan_record.example.json").read_text())
    assert contract_errors(example) == []


def test_mock_backend_is_idempotent(tmp_path):
    backend = MockBackend(output=tmp_path / "rows.jsonl")
    r = record()
    assert backend.upsert([r]) == [r["scan_id"]]
    assert backend.upsert([r]) == [r["scan_id"]]  # resend after a lost ack: no duplicate row
    assert len(backend.rows) == 1
    assert len((tmp_path / "rows.jsonl").read_text().splitlines()) == 1


def test_store_and_forward_survives_failures(tmp_path):
    q = ScanQueue(tmp_path / "q.sqlite")
    for i in range(30):
        q.put(record(code=f"SKU-{i}"))
    backend = MockBackend(failure_rate=0.5, seed=3)
    for _ in range(100):
        batch = q.pending(5)
        if not batch:
            break
        try:
            q.mark_synced(backend.upsert(batch))
        except BackendError as exc:
            q.mark_failed([r["scan_id"] for r in batch], str(exc))
    assert q.counts() == (30, 0)
    assert len(backend.rows) == 30


def test_http_backend_needs_credentials(monkeypatch):
    monkeypatch.delenv("INV_BACKEND_URL", raising=False)
    monkeypatch.delenv("INV_BACKEND_KEY", raising=False)
    with pytest.raises(BackendError, match="INV_BACKEND_URL"):
        HttpBackend()


def test_csv_export(tmp_path):
    path = export_csv([record(), record(code="SKU-2", position_map=None)], tmp_path / "out.csv")
    rows = list(csv.DictReader(path.open()))
    assert list(rows[0]) == COLUMNS
    assert [r["code"] for r in rows] == ["SKU-1", "SKU-2"]
    assert rows[0]["x"] == "1.0" and rows[1]["x"] == ""


@pytest.mark.parametrize(
    "name",
    ["../escape.csv", "/etc/passwd.csv", "a/b.csv", "..\\b.csv", "report.txt", "", ".csv", "..csv", "a b.csv"],
)
def test_export_refuses_anything_but_a_plain_csv_name(tmp_path, name):
    with pytest.raises(ValueError):
        resolve_export_path(tmp_path, name)


def test_export_accepts_a_plain_csv_name(tmp_path):
    assert resolve_export_path(tmp_path, "m1-20261106.csv") == tmp_path / "m1-20261106.csv"


def test_export_never_overwrites(tmp_path):
    existing = tmp_path / "out.csv"
    existing.write_text("keep me")
    with pytest.raises(FileExistsError):
        export_csv([record()], existing)
    assert existing.read_text() == "keep me"


def test_export_does_not_follow_a_planted_symlink(tmp_path):
    victim = tmp_path / "victim.txt"
    victim.write_text("keep me")
    link = tmp_path / "out.csv"
    try:
        link.symlink_to(victim)
    except OSError:
        pytest.skip("symlinks not available")
    with pytest.raises(FileExistsError):
        export_csv([record()], link)
    assert victim.read_text() == "keep me"


def test_mission_id_cannot_steer_the_export_name():
    assert "/" not in safe_name_part("../../etc/cron.d/x")
    assert not safe_name_part("..").startswith(".")


def _node_stub(queue, tmp_path, batch_size=2):
    ignore = lambda *_a, **_k: None  # noqa: E731
    logger = type("L", (), {"error": ignore, "warning": ignore, "info": ignore})()
    node = UplinkNode.__new__(UplinkNode)
    node.queue, node.backend = queue, MockBackend()
    node.p = {"batch_size": batch_size, "validate_records": True, "export_dir": str(tmp_path / "exports")}
    node.synced, node.last_error = 0, ""
    node.get_logger = lambda: logger
    return node


def test_service_export_stays_in_export_dir(tmp_path):
    q = ScanQueue(tmp_path / "q.sqlite")
    node = _node_stub(q, tmp_path)
    victim = tmp_path / "victim.csv"
    victim.write_text("keep me")
    with pytest.raises(ValueError):
        node.export(str(victim))
    assert victim.read_text() == "keep me"
    path, _ = node.export("", "../../evil")
    assert path.parent == tmp_path / "exports"
    q.close()


def test_export_neutralizes_spreadsheet_formulas(tmp_path):
    codes = ["=1+1", "+1", "-1", "@SUM(A1)", "\tx", "\rx"]
    path = export_csv([record(code=c) for c in codes], tmp_path / "out.csv")
    rows = list(csv.DictReader(path.open(newline="")))
    assert [r["code"] for r in rows] == [f"'{c}" for c in codes]


def test_export_leaves_ordinary_values_alone(tmp_path):
    negative = record(position_map=(-1.5, 2.0, 0.5))
    path = export_csv([negative], tmp_path / "out.csv")
    row = next(csv.DictReader(path.open()))
    assert row["code"] == "SKU-1" and row["x"] == "-1.5"  # numbers stay numbers


def test_invalid_records_cannot_starve_valid_ones(tmp_path):
    q = ScanQueue(tmp_path / "q.sqlite")
    bad = [record(code=f"SKU-{i}") for i in range(4)]
    for r in bad:
        del r["drone_id"]
        q.put(r)
    good = record(code="GOOD")
    q.put(good)
    node = _node_stub(q, tmp_path, batch_size=2)
    for _ in range(4):
        node._sync()
    assert node.synced == 1 and [r["code"] for r in node.backend.rows.values()] == ["GOOD"]
    assert q.counts() == (5, 0) and q.rejected_count() == 4
    assert len(q.records()) == 5  # rejected records are kept for inspection
    q.close()


def test_transient_failures_are_retried_not_rejected(tmp_path):
    q = ScanQueue(tmp_path / "q.sqlite")
    q.put(record())
    q.mark_failed([q.pending(1)[0]["scan_id"]], "timeout")
    assert q.counts() == (1, 1) and q.rejected_count() == 0
    assert len(q.pending(1)) == 1
    q.close()


def test_queue_opens_a_database_from_before_rejected_at(tmp_path):
    path = tmp_path / "old.sqlite"
    db = sqlite3.connect(path)
    db.execute(
        "CREATE TABLE scans (scan_id TEXT PRIMARY KEY, mission_id TEXT NOT NULL, created_at TEXT NOT NULL, "
        "record_json TEXT NOT NULL, synced_at TEXT, attempts INTEGER NOT NULL DEFAULT 0, last_error TEXT)"
    )
    db.execute("INSERT INTO scans VALUES ('a', 'm', '0', '{\"scan_id\": \"a\"}', NULL, 0, NULL)")
    db.commit()
    db.close()
    q = ScanQueue(path)
    assert q.counts() == (1, 1)
    q.mark_rejected(["a"], "contract violation")
    assert q.counts() == (1, 0) and q.rejected_count() == 1
    q.close()


def test_config_matches_defaults():
    path = pathlib.Path(__file__).resolve().parents[1] / "config" / "uplink.yaml"
    assert yaml.safe_load(path.read_text())["uplink"]["ros__parameters"] == DEFAULTS
