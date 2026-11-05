## What and why

<!-- One or two sentences. Link the issue / test card / ADR. -->

## How it was tested

- [ ] `just test` passes
- [ ] `just sim-smoke` passes (if it touches drone/, sim/, deps/, docker/ or params)
- [ ] Ran it in `just sim` and looked at the result (say what you saw)
- [ ] Hardware: test card + flight log linked (if it changes vehicle behaviour)

## Checklist

- [ ] No `/ap/*` outside `inv_flight_interface`; new nodes declare parameters in `config/` and support `use_sim_time`
- [ ] Parameter file changes: `just params-check` passes and the reason is in the PR description
- [ ] Contract changes (`contracts/`): app team approved; `schema_version` bumped if needed
- [ ] A decision others will ask "why?" about has an ADR in `docs/adr/`
- [ ] Docs updated (`docs/interfaces.md` for new topics/frames)
- [ ] No secrets, rosbags or generated worlds committed
