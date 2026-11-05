# Bill of materials

The parts the vehicle is being built from (decision: docs/adr/0009-hardware-baseline.md). "Checked"
means against the vendor page or the pinned ArduPilot source on 2026-11-05. **Nothing has been
bench-tested or flown.**

## Listed parts

| Part | Role | Listed price | What was checked |
|---|---|---|---|
| [Holybro Pixhawk 6C + PM02](https://holybro.com/products/pixhawk-6c?variant=45417175580861) (plastic case) | Flight controller, power module | $180 (vendor page: $217.99) | Ships with PX4, so it must be flashed. Copter-4.7.2 with DDS builds with 333 KB of flash free. Two IMUs |
| [Jetson Orin Nano Super Developer Kit, 8 GB](https://marketplace.nvidia.com/en-us/enterprise/robotics-edge/jetson-orin-nano-super-developer-kit/) | Autonomy computer | $400 | Isaac ROS 4.6 lists "Jetson Orin" on JetPack 7.2 with a 128+ GB NVMe; it does not name the Nano 8 GB |
| [Luxonis OAK-D Lite](https://shop.luxonis.com/products/oak-d-lite-1) | Stereo, depth, RGB, IMU | $270 | Global-shutter stereo, 640x480, 75 mm baseline; 13 MP RGB; BMI270 IMU. Sold as auto-focus or fixed-focus. **Not in Isaac ROS's documented cameras** |
| [Holybro X500 V2 ARF kit](https://holybro.com/products/x500-v2-kits?variant=42541212041405) | Frame, motors, ESCs, PDB, props | $329 | Price and contents match. Its optional camera mount is for RealSense and Structure Core, not OAK |
| [Battery (Zeee, Amazon)](https://www.amazon.com/dp/B088RFVCDF) | Flight battery | $140 | **Not checked**: the listing could not be read. Holybro recommends 4S, 3000-5000 mAh, XT60 |
| [Holybro SiK Telemetry Radio V3, 915 MHz](https://holybro.com/products/sik-telemetry-radio-v3?variant=41562952302781) | Ground-station link | $59 | The linked variant is 100 mW; Holybro's 500 mW variant is discontinued |
| [SiK radio, 500 mW (Motionew)](https://www.motionew.com/shop/data-link-video-link/telemetry-radio/sik-radio-telemetry/) | Alternative to the above | $66 | Page could not be read. Only one radio is needed |

The listed total of $1,444 counts both radios. With one radio it is $1,378 to $1,385, plus about
$38 at the Pixhawk's current vendor price.

## Needed but not listed

| Missing | Why it matters | Effect |
|---|---|---|
| **RC transmitter + receiver** | The safety pilot's mode switch and motor kill are mandatory (`flight_test/checklist.md`) | Blocks flight |
| **NVMe SSD** | Isaac ROS asks for 128+ GB NVMe | Blocks Isaac ROS |
| **Jetson power from the battery** | PM02 powers only the FC. Check the dev kit's input range in NVIDIA's carrier-board specification, then choose a regulator or a direct feed from the PDB | Blocks flight |
| **Downward rangefinder** | Recommended EKF height source; without one, height comes from VSLAM (`real.parm`) | Worked around |
| Optical flow sensor | Fallback when VSLAM is lost; without it the vehicle can only land (`FS_EKF_ACTION 1`) | Accepted for now |
| OAK-D Lite bracket | No vendor mount on this frame; needs vibration isolation | Custom part |
| FC-to-Jetson serial cable, LiPo charger, spare props, prop guards | Bench and indoor flight basics | Buy |

## Choices these parts force

- **Prefer the fixed-focus OAK-D Lite.** Luxonis recommends fixed focus for drones because of
  vibration, and it is sharp from 50 cm outward; the scan standoff is 1.0 m. On an auto-focus unit,
  lock the lens (`sensors/oak_d_lite.yaml`) and calibrate at that position.
- **Test VSLAM on this camera first** (`sensors/README.md`). If it does not hold up, the swappable
  localization interface means only the backend changes.
- **Weigh the vehicle.** All-up weight is unverified. Check hover throttle on the first tethered
  flight.
