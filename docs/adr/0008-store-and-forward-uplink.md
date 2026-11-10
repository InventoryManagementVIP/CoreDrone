# 0008: Store-and-forward uplink through SQLite

- Status: Accepted
- Date: 2026-10-20

## Context

The process document had the drone "continuously sending scans wirelessly to a web endpoint".
Warehouse Wi-Fi is unreliable (metal racks), and flight must never block on the network. Results
go to a Supabase/Postgres backend that the app team owns.

## Decision

- The inventory node writes each scan record to a **local SQLite queue** (WAL mode) on the Jetson.
- A separate **uplink** node syncs pending records whenever it can: idempotent upserts keyed by the
  record's UUID `scan_id`, batches of 50, retries forever, never on the flight path. It also
  exports the CSV the process document asks for.
- The record format is a versioned JSON Schema in `contracts/scan_record.schema.json`. This repo
  owns the format; the app team owns the database. Changes need approval from both CODEOWNERS
  groups. Records are validated against the schema before upload, and in unit tests.
- Credentials come from environment files that are never committed (`.env.example`); use an
  insert-only key, never the service-role key.

## Alternatives considered

- **Stream directly to the backend during flight**: loses data on every Wi-Fi gap and couples
  flight to the network. Rejected.
- **ROS bag on the drone, process after landing**: no live progress and needs offline tooling.
  Rejected for the records (bags are still recorded for debugging).

## Consequences

- A mock backend (with simulated failures) makes the whole path testable in CI.
- The HTTP backend is a stub until the app team signs off the table schema (phase 2).
- The drone records an item code once per label-attributed slot per mission, plus weaker reads
  only while nothing better exists (contracts/README.md has the box-counting rule). Each record has
  its own `scan_id`. Merging corrections into records already queued is phase 2.

## Revisit when

The backend contract is signed off (implement the HTTP client), or reads must reach the app during
the flight (add a live, best-effort channel *next to* the queue, not instead of it).
