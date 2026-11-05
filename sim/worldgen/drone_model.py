"""Generate the simulated CoreDrone model and its ros_gz_bridge config.

* Airframe, rotors, LiftDrag/ApplyJointForce and the ArduPilotPlugin come from ardupilot_gazebo's
  pinned iris_with_ardupilot model (copied, not hand-maintained).
* Sensor poses come from the URDF (inv_description); intrinsics from sim/models/inv_quad/sensors.yaml.
* The downward rangefinder is always present and wired into ArduPilotPlugin, so SITL fuses it
  exactly like the real vehicle (EK3_SRC1_POSZ = rangefinder).
* An OdometryPublisher provides the true pose for the `groundtruth` localization backend.
"""

from __future__ import annotations

import copy
import os
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

from worldgen.sdf import model_config, pose, sub, to_string

MODEL_NAME = "inv_quad"
BASE = "iris_with_standoffs::base_link"
SENSORS_YAML = Path(__file__).resolve().parents[1] / "models" / "inv_quad" / "sensors.yaml"
GZ_PREFIX = f"/{MODEL_NAME}"
GROUND_TRUTH_GZ_TOPIC = f"/model/{MODEL_NAME}/odometry"
GROUND_TRUTH_ROS_TOPIC = "/sim/ground_truth/odom"


def load_sensor_config(path: Path = SENSORS_YAML) -> dict:
    return yaml.safe_load(path.read_text())


def ardupilot_gazebo_share() -> Path:
    env = os.environ.get("ARDUPILOT_GAZEBO_SHARE")
    if env:
        return Path(env)
    from ament_index_python.packages import get_package_share_directory

    return Path(get_package_share_directory("ardupilot_gazebo"))


def iris_plugins(share: Path | None = None) -> list[ET.Element]:
    """The <plugin> elements of ardupilot_gazebo's iris_with_ardupilot model."""
    sdf = ET.parse((share or ardupilot_gazebo_share()) / "models" / "iris_with_ardupilot" / "model.sdf")
    model = sdf.getroot().find("model")
    plugins = [copy.deepcopy(p) for p in model.findall("plugin")]
    if not any(p.get("name") == "ArduPilotPlugin" for p in plugins):
        raise RuntimeError("iris_with_ardupilot has no ArduPilotPlugin: ardupilot_gazebo layout changed")
    return plugins


def _sensor_element(link: ET.Element, name: str, spec: dict) -> None:
    kind = spec["type"]
    sensor = sub(link, "sensor", name=name, type=kind)
    sub(sensor, "always_on", "1")
    sub(sensor, "update_rate", str(spec["rate_hz"]))
    sub(sensor, "visualize", "false")
    sub(sensor, "topic", f"{GZ_PREFIX}/sensor/{name}")
    sub(sensor, "gz_frame_id", spec["frame_id"])
    if kind in ("camera", "depth_camera"):
        cam = sub(sensor, "camera", name=name)
        sub(cam, "horizontal_fov", str(spec["horizontal_fov"]))
        image = sub(cam, "image")
        sub(image, "width", str(spec["width"]))
        sub(image, "height", str(spec["height"]))
        sub(image, "format", spec["format"])
        clip = sub(cam, "clip")
        sub(clip, "near", str(spec["clip"][0]))
        sub(clip, "far", str(spec["clip"][1]))
        sub(cam, "camera_info_topic", f"{GZ_PREFIX}/sensor/{name}/camera_info")
    elif kind == "gpu_lidar":
        lidar = sub(sensor, "lidar")
        scan = sub(lidar, "scan")
        h = sub(scan, "horizontal")
        sub(h, "samples", "1")
        sub(h, "resolution", "1")
        sub(h, "min_angle", "0")
        sub(h, "max_angle", "0")
        rng = sub(lidar, "range")
        sub(rng, "min", str(spec["range"][0]))
        sub(rng, "max", str(spec["range"][1]))
        sub(rng, "resolution", "0.01")


def build_drone_model(
    urdf_xml: str, sensor_set: str, models_dir: Path, sensor_config: dict | None = None, share: Path | None = None
) -> list[dict]:
    """Write models/inv_quad/. Returns the ros_gz_bridge entries for the chosen sensors."""
    from inv_description.extrinsics import frame_pose

    config = sensor_config or load_sensor_config()
    if sensor_set not in config["sensor_sets"]:
        raise ValueError(f"unknown sensor set {sensor_set!r}; choose from {', '.join(config['sensor_sets'])}")

    sdf = ET.Element("sdf", version="1.9")
    model = sub(sdf, "model", name=MODEL_NAME)
    include = sub(model, "include")
    sub(include, "uri", "model://iris_with_standoffs")

    bridge: list[dict] = [
        {
            "ros_topic_name": "/clock",
            "gz_topic_name": "/clock",
            "ros_type_name": "rosgraph_msgs/msg/Clock",
            "gz_type_name": "gz.msgs.Clock",
            "direction": "GZ_TO_ROS",
        },
        {
            "ros_topic_name": GROUND_TRUTH_ROS_TOPIC,
            "gz_topic_name": GROUND_TRUTH_GZ_TOPIC,
            "ros_type_name": "nav_msgs/msg/Odometry",
            "gz_type_name": "gz.msgs.Odometry",
            "direction": "GZ_TO_ROS",
        },
    ]

    ardupilot_ranges: list[tuple[int, str]] = []
    for name in config.get("flight_sensors", []) + config["sensor_sets"][sensor_set]:
        spec = config["sensors"][name]
        if "ardupilot_rangefinder_index" in spec:
            ardupilot_ranges.append((int(spec["ardupilot_rangefinder_index"]), f"{GZ_PREFIX}/sensor/{name}"))
        fp = frame_pose(urdf_xml, spec["link"])
        link = sub(model, "link", name=f"{name}_sensor_link")
        pose(link, *fp.xyz, *fp.rpy, relative_to=BASE)
        inertial = sub(link, "inertial")
        sub(inertial, "mass", "0.001")
        inertia = sub(inertial, "inertia")
        for axis in ("ixx", "iyy", "izz"):
            sub(inertia, axis, "1e-07")
        _sensor_element(link, name, spec)
        joint = sub(model, "joint", name=f"{name}_sensor_joint", type="fixed")
        sub(joint, "parent", BASE)
        sub(joint, "child", f"{name}_sensor_link")

        gz_topic = f"{GZ_PREFIX}/sensor/{name}"
        if spec["type"] in ("camera", "depth_camera"):
            bridge.append(
                {
                    "ros_topic_name": spec["ros_topic"],
                    "gz_topic_name": gz_topic,
                    "ros_type_name": "sensor_msgs/msg/Image",
                    "gz_type_name": "gz.msgs.Image",
                    "direction": "GZ_TO_ROS",
                }
            )
            bridge.append(
                {
                    "ros_topic_name": spec["ros_info_topic"],
                    "gz_topic_name": f"{gz_topic}/camera_info",
                    "ros_type_name": "sensor_msgs/msg/CameraInfo",
                    "gz_type_name": "gz.msgs.CameraInfo",
                    "direction": "GZ_TO_ROS",
                }
            )
        elif spec["type"] == "imu":
            bridge.append(
                {
                    "ros_topic_name": spec["ros_topic"],
                    "gz_topic_name": gz_topic,
                    "ros_type_name": "sensor_msgs/msg/Imu",
                    "gz_type_name": "gz.msgs.IMU",
                    "direction": "GZ_TO_ROS",
                }
            )
        elif spec["type"] == "gpu_lidar":
            bridge.append(
                {
                    "ros_topic_name": spec["ros_topic"],
                    "gz_topic_name": gz_topic,
                    "ros_type_name": "sensor_msgs/msg/LaserScan",
                    "gz_type_name": "gz.msgs.LaserScan",
                    "direction": "GZ_TO_ROS",
                }
            )

    for plugin in iris_plugins(share):
        if plugin.get("name") == "ArduPilotPlugin":
            # Range sensors ArduPilotPlugin forwards to SITL as JSON rng_<index>.
            for index, topic in ardupilot_ranges:
                rng = sub(plugin, "sensor")
                sub(rng, "type", "RANGEFINDER")
                sub(rng, "index", str(index))
                sub(rng, "topic", topic)
        model.append(plugin)
    odom = sub(model, "plugin", filename="gz-sim-odometry-publisher-system", name="gz::sim::systems::OdometryPublisher")
    sub(odom, "odom_frame", "map")
    sub(odom, "robot_base_frame", "base_link")
    sub(odom, "odom_publish_frequency", "50")
    sub(odom, "odom_topic", GROUND_TRUTH_GZ_TOPIC)
    sub(odom, "dimensions", "3")

    out = models_dir / MODEL_NAME
    out.mkdir(parents=True, exist_ok=True)
    (out / "model.sdf").write_text(to_string(sdf))
    (out / "model.config").write_text(
        model_config(MODEL_NAME, f"CoreDrone quad (iris-derived), sensor set '{sensor_set}'. Generated; do not edit.")
    )
    return bridge


# iris_with_standoffs base_link height when the model rests on its legs (as in ardupilot_gazebo's worlds).
SPAWN_HEIGHT_M = 0.195
