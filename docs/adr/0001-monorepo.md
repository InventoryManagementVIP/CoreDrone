# 0001: Monorepo with hard boundaries

- Status: Accepted
- Date: 2026-10-20

## Context

The previous repo mixed experiments with platform code and was hard to install. The team now has
four kinds of work that change together: the ROS 2 autonomy stack, the simulator, the vehicle's
platform configuration, and the data contract with the backend. A separate team builds the React +
Supabase Companion App and has its own momentum.

## Decision

One repository with hard top-level boundaries:

- `drone/`: the ROS 2 overlay workspace; all autonomy code lives here exactly once
- `sim/` and `real/`: platform-specific things only (worlds, models, drivers, deployment,
  parameter overrides). If a node gets copied into `sim/` or `real/`, that is a bug.
- `contracts/`: the drone → backend record format and the warehouse layout schema
- `app/`: reserved. The Companion App stays in its own repository for now; nothing here depends
  on it, and `app/README.md` points to it.
- `docs/` (including these ADRs), `docker/`, `deps/`, `config/`.

## Alternatives considered

- **Separate repos for drone, sim and real**: the sim and the stack must change in the same PR
  (topic names, launch arguments), so split repos would drift. Rejected.
- **Move the Companion App in now**: no shared code yet, and the app team's workflow would be
  disrupted. Deferred: the contract in `contracts/` is the only coupling, and changes to it need
  approval from both CODEOWNERS groups.

## Consequences

- One clone, one `just setup`, one CI.
- CODEOWNERS per subsystem keeps reviews with the right people as students rotate.
- CI must path-filter expensive jobs (`sim-smoke` only runs when sim-relevant paths change).

## Revisit when

The app team wants shared code or shared CI with the drone (then migrate it into `app/` with its
own CODEOWNERS), or the repo becomes too slow to clone (then split out large assets first).
