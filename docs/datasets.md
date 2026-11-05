# Datasets and recordings

**Never commit rosbags.** Record with `just record PROFILE=...` (MCAP; profiles in
`drone/src/inv_tools/config/bag_topics.yaml`), upload to the team's shared storage, and add a row
here so others can find it.

| Date | Name | Where | Profile | Platform | What it shows | Who |
|---|---|---|---|---|---|---|
| | | | | | | |

Conventions:
- Name: `<yyyymmdd>_<warehouse>_<what>`, e.g. `20261103_lab_aisle1_vslam_drift`.
- Storage: TODO(team): pick the shared drive / bucket and link it here.
- Say which commit of this repo produced it (`git rev-parse --short HEAD`) and any parameter overrides.
- Sim bags are only worth keeping when they reproduce a bug; real flight bags are always worth keeping.
