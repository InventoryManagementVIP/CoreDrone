"""Backend clients for the uplink. The node only talks to the BackendClient interface.

  mock   in-memory (optionally appending to a JSONL file); can simulate an unreliable network.
         Used in simulation and tests.
  http   Supabase/PostgREST upsert keyed by scan_id. TODO(phase-2): needs contract sign-off with
         the app team (table name, row-level security, which key the drone uses).

Credentials never live in the repo: they come from environment variables (see .env.example),
which on the Jetson are loaded from a root-only env file by systemd.
"""

from __future__ import annotations

import json
import os
import random
from abc import ABC, abstractmethod
from pathlib import Path


class BackendError(RuntimeError):
    pass


class BackendClient(ABC):
    name = "abstract"

    @abstractmethod
    def upsert(self, records: list[dict]) -> list[str]:
        """Send records; return the scan_ids the backend has durably accepted. Must be idempotent."""


class MockBackend(BackendClient):
    name = "mock"

    def __init__(self, output: str | Path | None = None, failure_rate: float = 0.0, seed: int = 0):
        self.rows: dict[str, dict] = {}
        self.output = Path(output) if output else None
        self.failure_rate = failure_rate
        self._rng = random.Random(seed)
        if self.output:
            self.output.parent.mkdir(parents=True, exist_ok=True)

    def upsert(self, records: list[dict]) -> list[str]:
        if self.failure_rate and self._rng.random() < self.failure_rate:
            raise BackendError("simulated network failure")
        accepted = []
        for record in records:
            new = record["scan_id"] not in self.rows
            self.rows[record["scan_id"]] = record
            if new and self.output:
                with self.output.open("a") as f:
                    f.write(json.dumps(record, sort_keys=True) + "\n")
            accepted.append(record["scan_id"])
        return accepted


class HttpBackend(BackendClient):
    """Supabase REST upsert. TODO(phase-2): not enabled until the app team signs off the contract."""

    name = "http"
    URL_ENV = "INV_BACKEND_URL"  # e.g. https://<project>.supabase.co/rest/v1/scan_records
    KEY_ENV = "INV_BACKEND_KEY"  # an insert-only key, never the service-role key

    def __init__(self, timeout_s: float = 5.0):
        self.url = os.environ.get(self.URL_ENV, "")
        self.key = os.environ.get(self.KEY_ENV, "")
        self.timeout_s = timeout_s
        if not self.url or not self.key:
            raise BackendError(f"set {self.URL_ENV} and {self.KEY_ENV} (see .env.example)")

    def upsert(self, records: list[dict]) -> list[str]:
        raise BackendError(
            "http backend is TODO(phase-2): POST to INV_BACKEND_URL with 'Prefer: resolution=merge-duplicates' "
            "and on_conflict=scan_id, once the table schema is agreed with the app team"
        )


def make_backend(kind: str, **kwargs) -> BackendClient:
    if kind == "mock":
        return MockBackend(**kwargs)
    if kind == "http":
        return HttpBackend()
    raise ValueError(f"unknown uplink backend {kind!r}")
