"""Store-and-forward queue for scan records (SQLite on the Jetson).

Flight never waits on the network (docs/adr/0008-store-and-forward-uplink.md): the inventory
node only writes here; inv_uplink syncs to the backend whenever it can, idempotently by scan_id.
WAL mode lets the writer and the uplink reader work at the same time.
"""

from __future__ import annotations

import datetime as dt
import json
import sqlite3
from pathlib import Path

_SCHEMA = """
CREATE TABLE IF NOT EXISTS scans (
    scan_id     TEXT PRIMARY KEY,
    mission_id  TEXT NOT NULL,
    created_at  TEXT NOT NULL,
    record_json TEXT NOT NULL,
    synced_at   TEXT,
    attempts    INTEGER NOT NULL DEFAULT 0,
    last_error  TEXT
);
CREATE INDEX IF NOT EXISTS scans_pending ON scans (synced_at, created_at);
"""


def _now() -> str:
    return dt.datetime.now(dt.timezone.utc).isoformat(timespec="milliseconds")


class ScanQueue:
    def __init__(self, path: str | Path):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._db = sqlite3.connect(str(self.path), timeout=10.0, check_same_thread=False)
        self._db.execute("PRAGMA journal_mode=WAL")
        self._db.execute("PRAGMA synchronous=NORMAL")
        self._db.executescript(_SCHEMA)
        self._db.commit()

    def close(self) -> None:
        self._db.close()

    def put(self, record: dict) -> bool:
        """Insert a record. Returns False if its scan_id is already queued (idempotent)."""
        cur = self._db.execute(
            "INSERT OR IGNORE INTO scans (scan_id, mission_id, created_at, record_json) VALUES (?, ?, ?, ?)",
            (record["scan_id"], record["mission_id"], _now(), json.dumps(record, sort_keys=True)),
        )
        self._db.commit()
        return cur.rowcount == 1

    def pending(self, limit: int = 100) -> list[dict]:
        rows = self._db.execute(
            "SELECT record_json FROM scans WHERE synced_at IS NULL ORDER BY created_at LIMIT ?", (limit,)
        ).fetchall()
        return [json.loads(r[0]) for r in rows]

    def mark_synced(self, scan_ids: list[str]) -> None:
        now = _now()
        self._db.executemany(
            "UPDATE scans SET synced_at = ?, last_error = NULL WHERE scan_id = ?", [(now, s) for s in scan_ids]
        )
        self._db.commit()

    def mark_failed(self, scan_ids: list[str], error: str) -> None:
        self._db.executemany(
            "UPDATE scans SET attempts = attempts + 1, last_error = ? WHERE scan_id = ?",
            [(error[:500], s) for s in scan_ids],
        )
        self._db.commit()

    def records(self, mission_id: str | None = None) -> list[dict]:
        if mission_id:
            rows = self._db.execute(
                "SELECT record_json FROM scans WHERE mission_id = ? ORDER BY created_at", (mission_id,)
            ).fetchall()
        else:
            rows = self._db.execute("SELECT record_json FROM scans ORDER BY created_at").fetchall()
        return [json.loads(r[0]) for r in rows]

    def counts(self) -> tuple[int, int]:
        """(total, pending)."""
        total = self._db.execute("SELECT COUNT(*) FROM scans").fetchone()[0]
        pending = self._db.execute("SELECT COUNT(*) FROM scans WHERE synced_at IS NULL").fetchone()[0]
        return total, pending
