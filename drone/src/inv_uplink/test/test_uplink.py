import csv
import datetime as dt
import json
import pathlib

import pytest
import yaml

from inv_inventory.queue import ScanQueue
from inv_inventory.records import make_scan_record
from inv_uplink.backends import BackendError, HttpBackend, MockBackend
from inv_uplink.export import COLUMNS, contract_errors, export_csv
from inv_uplink.node import DEFAULTS

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


def test_config_matches_defaults():
    path = pathlib.Path(__file__).resolve().parents[1] / "config" / "uplink.yaml"
    assert yaml.safe_load(path.read_text())["uplink"]["ros__parameters"] == DEFAULTS
