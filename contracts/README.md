# contracts/: data formats shared across teams and subsystems

| File | What | Owners |
|---|---|---|
| `scan_record.schema.json` | one item read, drone → backend (JSON Schema 2020-12) | this repo (format) + app team (database) |
| `warehouse_layout.schema.json` | `config/warehouses/*.yaml` (racks, slots, labels, sim contents) | this repo |
| `examples/` | valid examples, checked in CI | |
| `validate.py` | checks the schemas, examples and every layout (`just test-sim`, CI `contracts` job) | |

## Scan record rules

- `scan_id` is a UUID generated on the drone; the backend **upserts on it**, so resending is safe
  (docs/adr/0008-store-and-forward-uplink.md).
- `schema_version` is `"<major>.<minor>"`. Minor versions only add optional fields; anything else is
  a new major version, coordinated with the app team before merging.
- Changes need approval from **both** CODEOWNERS groups (drone and app).
- The uplink validates every record against this schema before sending; unit tests do too.
- `code` names a product, not a physical box. Within one mission the drone records a code once per
  slot it is label-attributed to, plus at most one `pose` and one `none` read while nothing better
  exists. To count boxes: one per distinct `slot_code` among a code's `label` records; if it has
  none, one for its best `pose` (else `none`) record. Boxes of one product side by side in one slot
  count as one (`sim/evaluation/score.py` `boxes()` implements this).

## Draft fields (v1.0)

`scan_id`, `mission_id`, `drone_id`, `warehouse_id`, `timestamp` (RFC 3339 UTC), `code`,
`symbology` (`qr` | `ean13` | `code128` | `apriltag` | `other`), `slot_code`, `rack_id`, `level`,
`position_map` {x, y, z} in the warehouse map frame, `confidence` [0, 1], `attribution_method`
(`label` | `pose` | `none`), `image_ref` (optional).

Open questions for the app team: table name and primary key, which key the drone uses (it must be
insert-only), whether `image_ref` points to Supabase Storage, retention of `attribution_method: none`
records.
