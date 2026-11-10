# Pre-flight checklist

Every flight, every time. Two people minimum: a **safety pilot** on the RC transmitter and an
**operator** at the laptop. The safety pilot's mode switch and motor kill switch always win: the
stack stops commanding the moment the mode changes (`pilot_override` in `/flight/state`).

## Before going to the flight area
- [ ] Test card written and reviewed (`test_card_template.md`)
- [ ] Batteries charged (flight + RC + laptop); battery voltage recorded
- [ ] `git rev-parse --short HEAD` recorded; `just params-diff` shows no unexpected differences
- [ ] Layout file for this site exists and its `geodetic_origin` is set
- [ ] Location labels / AprilTags on the racks for the area being flown

## On site, props OFF
- [ ] Area cleared; people outside the net/boundary; fire extinguisher (LiPo) at hand
- [ ] RC transmitter on first; mode switch in LOITER (or ALT_HOLD); kill switch tested
- [ ] Jetson boots; `systemctl status coredrone` active; `/flight/state.connected` true
- [ ] `fc_bootstrap` log: DDS initialization passed, EKF origin set
- [ ] `/localization/status` TRACKING; carry the drone by hand and check the pose moves correctly in Foxglove
- [ ] GCS: `EKF3 ... is using external nav data`; no pre-arm errors
- [ ] Losing VSLAM on the bench (stop the camera, props off): the stack reports LOST and requests a landing. No flow sensor on the BOM, so there is no position fallback
- [ ] Geofence box from the layout matches the area (`flight_interface` log)
- [ ] Recording started (`just record PROFILE=minimal`)

## Props ON
- [ ] Safety pilot arms in LOITER, takes off by hand to 1 m, checks hold, lands, disarms
- [ ] Then, if the test card allows: switch to GUIDED / start the mission from the laptop
- [ ] Safety pilot keeps hand on the mode switch for the whole autonomous segment

## After
- [ ] Disarmed, battery disconnected
- [ ] Logs collected: dataflash log from the FC, bag, `runs/` directory; list them in docs/datasets.md
- [ ] Flight log filled in (`flight_log_template.md`), including anything surprising
