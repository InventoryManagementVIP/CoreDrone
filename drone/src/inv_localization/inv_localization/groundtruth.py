"""`groundtruth` localization backend (simulation only).

Takes Gazebo's true pose of the drone (bridged to /sim/ground_truth/odom) and publishes it the way
a VSLAM backend would: TF map->odom->base_link and nav_msgs/Odometry on /localization/odom.
Optional seeded noise, drift and dropouts make it behave more like VSLAM (config/groundtruth.yaml).

This is what Mac, Windows and CI developers use instead of Isaac ROS, which needs CUDA.
"""

from __future__ import annotations

import math

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from inv_interfaces.msg import LocalizationStatus
from inv_localization.noise import NoiseConfig, NoiseModel, in_dropout, parse_dropouts
from inv_localization.transforms import matrix_to_quat, pose_to_matrix, rot_z

DEFAULTS: dict[str, object] = {
    "input_topic": "/sim/ground_truth/odom",
    "map_frame": "map",
    "odom_frame": "odom",
    "base_frame": "base_link",
    "position_noise_m": 0.0,
    "yaw_noise_rad": 0.0,
    "drift_m_per_sqrt_s": 0.0,
    "yaw_drift_rad_per_sqrt_s": 0.0,
    "seed": 0,
    "dropouts": "",
    "status_rate_hz": 5.0,
}


class GroundTruthBackend(Node):
    def __init__(self) -> None:
        super().__init__("localization_groundtruth")
        for name, value in DEFAULTS.items():
            self.declare_parameter(name, value)
        p = {name: self.get_parameter(name).value for name in DEFAULTS}
        self.p = p
        self.noise = NoiseModel(
            NoiseConfig(
                position_noise_m=p["position_noise_m"],
                yaw_noise_rad=p["yaw_noise_rad"],
                drift_m_per_sqrt_s=p["drift_m_per_sqrt_s"],
                yaw_drift_rad_per_sqrt_s=p["yaw_drift_rad_per_sqrt_s"],
                seed=int(p["seed"]),
            )
        )
        self.dropouts = parse_dropouts(p["dropouts"])
        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, "/localization/odom", 20)
        self.status_pub = self.create_publisher(LocalizationStatus, "/localization/backend_status", 10)
        self.create_subscription(Odometry, p["input_topic"], self._on_truth, qos_profile_sensor_data)
        self.create_timer(1.0 / p["status_rate_hz"], self._publish_status)
        self._tracking = False
        self._publish_map_to_odom()
        noise = "with noise/drift" if self.noise.config.enabled else "exact"
        self.get_logger().info(f"groundtruth backend: {p['input_topic']} -> /localization/odom ({noise})")

    def _publish_map_to_odom(self) -> None:
        # No loop closures to model: map and odom coincide. Drift therefore shows up in map too.
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.p["map_frame"]
        t.child_frame_id = self.p["odom_frame"]
        t.transform.rotation.w = 1.0
        self.static_tf.sendTransform(t)

    def _on_truth(self, msg: Odometry) -> None:
        stamp_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if in_dropout(stamp_s, self.dropouts):
            self._tracking = False
            return
        self._tracking = True
        m = pose_to_matrix(msg.pose.pose.position, msg.pose.pose.orientation)
        if self.noise.config.enabled:
            err_xyz, err_yaw = self.noise.sample(stamp_s)
            m = rot_z(err_yaw) @ m
            m[:3, 3] = m[:3, 3] + err_xyz
        qx, qy, qz, qw = matrix_to_quat(m[:3, :3])

        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.p["odom_frame"]
        t.child_frame_id = self.p["base_frame"]
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = (float(v) for v in m[:3, 3])
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = qx, qy, qz, qw
        self.tf.sendTransform(t)

        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.p["odom_frame"]
        out.child_frame_id = self.p["base_frame"]
        out.pose.pose.position.x, out.pose.pose.position.y, out.pose.pose.position.z = (float(v) for v in m[:3, 3])
        out.pose.pose.orientation = t.transform.rotation
        var = max(self.noise.config.position_noise_m, 1e-3) ** 2
        cov = np.diag([var, var, var, 1e-4, 1e-4, max(self.noise.config.yaw_noise_rad, 1e-3) ** 2])
        out.pose.covariance = [float(v) for v in cov.flatten()]
        out.twist = msg.twist
        self.odom_pub.publish(out)

    def _publish_status(self) -> None:
        msg = LocalizationStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.backend = "groundtruth"
        msg.state = LocalizationStatus.STATE_TRACKING if self._tracking else LocalizationStatus.STATE_LOST
        msg.rate_hz = math.nan
        msg.age_s = math.nan
        msg.message = "" if self._tracking else "no ground truth (waiting for sim, or simulated dropout)"
        self.status_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GroundTruthBackend()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
