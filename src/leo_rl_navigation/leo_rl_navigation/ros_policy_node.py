"""Deploy a trained PPO policy through the Leo Rover ROS 2 interface."""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from stable_baselines3 import PPO
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker

from leo_rl_navigation.policy_io import (
    action_to_command,
    apply_footprint_safety,
    build_observation,
)


def _yaw_from_odometry(message: Odometry) -> float:
    quaternion = message.pose.pose.orientation
    sin_yaw = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cos_yaw = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    return math.atan2(sin_yaw, cos_yaw)


class RosPolicyNode(Node):
    """Run deterministic PPO inference from ROS LiDAR and odometry."""

    def __init__(self) -> None:
        super().__init__('leo_rl_policy_node')

        self.model_path = Path(
            self.declare_parameter('model_path', '').value
        ).expanduser().resolve()
        self.goal_x = float(self.declare_parameter('goal_x', 0.0).value)
        self.goal_y = float(self.declare_parameter('goal_y', 0.0).value)
        self.n_sectors = int(self.declare_parameter('n_sectors', 50).value)
        self.number_of_rays = int(
            self.declare_parameter('number_of_rays', 500).value)
        self.range_min = float(
            self.declare_parameter('range_min', 0.05).value)
        self.range_max = float(
            self.declare_parameter('range_max', 12.0).value)
        self.maximum_goal_distance = float(
            self.declare_parameter(
                'maximum_goal_distance', math.sqrt(200.0)).value)
        self.front_half_angle_deg = float(
            self.declare_parameter('front_half_angle_deg', 30.0).value)
        self.include_front_clearance = bool(
            self.declare_parameter(
                'include_front_clearance', False).value)
        self.front_normalization_distance = float(
            self.declare_parameter(
                'front_normalization_distance', 0.80).value)
        self.include_directional_clearance = bool(
            self.declare_parameter(
                'include_directional_clearance', False).value)
        self.directional_normalization_distance = float(
            self.declare_parameter(
                'directional_normalization_distance', 3.0).value)
        self.directional_inner_angle_deg = float(
            self.declare_parameter(
                'directional_inner_angle_deg', 25.0).value)
        self.directional_outer_angle_deg = float(
            self.declare_parameter(
                'directional_outer_angle_deg', 85.0).value)
        self.use_footprint_clearance = bool(
            self.declare_parameter('use_footprint_clearance', False).value)
        self.footprint_half_length = float(
            self.declare_parameter('footprint_half_length', 0.215).value)
        self.footprint_half_width = float(
            self.declare_parameter('footprint_half_width', 0.259).value)
        self.lidar_offset_x = float(
            self.declare_parameter('lidar_offset_x', 0.10).value)
        self.lidar_offset_y = float(
            self.declare_parameter('lidar_offset_y', 0.0).value)
        self.footprint_half_angle_deg = float(
            self.declare_parameter('footprint_half_angle_deg', 90.0).value)
        self.enable_safety_shield = bool(
            self.declare_parameter('enable_safety_shield', False).value)
        self.safety_stop_distance = float(
            self.declare_parameter('safety_stop_distance', 0.08).value)
        self.safety_slowdown_distance = float(
            self.declare_parameter('safety_slowdown_distance', 0.35).value)
        self.safety_minimum_turn_speed = float(
            self.declare_parameter('safety_minimum_turn_speed', 0.40).value)
        self.linear_speed_max = float(
            self.declare_parameter('linear_speed_max', 0.25).value)
        self.linear_reverse_speed_max = float(
            self.declare_parameter('linear_reverse_speed_max', 0.0).value)
        self.angular_speed_max = float(
            self.declare_parameter('angular_speed_max', 0.80).value)
        self.goal_tolerance = float(
            self.declare_parameter('goal_tolerance', 0.25).value)
        self.maximum_episode_steps = int(
            self.declare_parameter('maximum_episode_steps', 400).value)
        self.control_hz = float(
            self.declare_parameter('control_hz', 10.0).value)
        self.sensor_timeout = float(
            self.declare_parameter('sensor_timeout', 0.50).value)

        if not self.model_path.is_file():
            raise FileNotFoundError(f'Model does not exist: {self.model_path}')
        if self.control_hz <= 0.0 or self.sensor_timeout <= 0.0:
            raise ValueError('Control frequency and sensor timeout must be positive')

        self.model = PPO.load(str(self.model_path), device='cpu')
        expected_observation_size = (
            self.n_sectors + 3 + int(self.include_front_clearance) +
            2 * int(self.include_directional_clearance))
        if self.model.observation_space.shape != (expected_observation_size,):
            raise ValueError(
                'Model observation shape does not match the ROS configuration: '
                f'{self.model.observation_space.shape} versus '
                f'{(expected_observation_size,)}')
        if self.model.action_space.shape != (2,):
            raise ValueError(
                f'Expected a two-value action, got {self.model.action_space.shape}')

        self.command_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        marker_qos = QoSProfile(depth=1)
        marker_qos.reliability = ReliabilityPolicy.RELIABLE
        marker_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.goal_publisher = self.create_publisher(
            Marker, '/rl_goal', marker_qos)

        self.create_subscription(LaserScan, '/scan', self._scan_callback, 10)
        self.create_subscription(Odometry, '/odom', self._odom_callback, 10)
        self.create_subscription(Bool, '/collision', self._collision_callback, 10)

        self.latest_scan: LaserScan | None = None
        self.latest_odometry: Odometry | None = None
        self.last_scan_time = None
        self.last_odometry_time = None
        self.last_processed_scan_stamp: tuple[int, int] | None = None
        self.collision = False
        self.finished = False
        self.episode_step = 0
        self.waiting_logged = False

        self.timer = self.create_timer(1.0 / self.control_hz, self._control)
        self._publish_goal_marker()
        self.get_logger().info(f'Loaded PPO model: {self.model_path}')
        self.get_logger().info(
            f'Goal: ({self.goal_x:.6f}, {self.goal_y:.6f}); '
            f'control={self.control_hz:.1f} Hz; '
            f'footprint_clearance={self.use_footprint_clearance}; '
            f'directional_clearance={self.include_directional_clearance}; '
            f'reverse_max={self.linear_reverse_speed_max:.3f}; '
            f'safety_shield={self.enable_safety_shield}')

    def _scan_callback(self, message: LaserScan) -> None:
        self.latest_scan = message
        self.last_scan_time = self.get_clock().now()

    def _odom_callback(self, message: Odometry) -> None:
        self.latest_odometry = message
        self.last_odometry_time = self.get_clock().now()

    def _collision_callback(self, message: Bool) -> None:
        self.collision = bool(message.data)

    def _publish_stop(self) -> None:
        self.command_publisher.publish(Twist())

    def _inputs_are_fresh(self) -> bool:
        if self.last_scan_time is None or self.last_odometry_time is None:
            return False
        timeout = Duration(seconds=self.sensor_timeout)
        current = self.get_clock().now()
        return (
            current - self.last_scan_time <= timeout and
            current - self.last_odometry_time <= timeout
        )

    def _control(self) -> None:
        self._publish_goal_marker()
        if self.finished:
            self._publish_stop()
            return

        if self.latest_scan is None or self.latest_odometry is None:
            self._publish_stop()
            if not self.waiting_logged:
                self.get_logger().info('Waiting for /scan and /odom...')
                self.waiting_logged = True
            return

        if not self._inputs_are_fresh():
            self._publish_stop()
            self.get_logger().warning(
                'Sensor data is stale; publishing zero velocity',
                throttle_duration_sec=2.0,
            )
            return

        stamp = self.latest_scan.header.stamp
        scan_stamp = (stamp.sec, stamp.nanosec)
        if scan_stamp == self.last_processed_scan_stamp:
            return
        odometry_stamp = self.latest_odometry.header.stamp
        if (odometry_stamp.sec, odometry_stamp.nanosec) < scan_stamp:
            return
        self.last_processed_scan_stamp = scan_stamp
        self.waiting_logged = False

        if self.collision:
            self.finished = True
            self._publish_stop()
            self.get_logger().error(
                f'Collision at policy step {self.episode_step}; stopped')
            return

        position = self.latest_odometry.pose.pose.position
        yaw = _yaw_from_odometry(self.latest_odometry)
        observation, measurements = build_observation(
            self.latest_scan.ranges,
            pose_x=position.x,
            pose_y=position.y,
            pose_yaw=yaw,
            goal_x=self.goal_x,
            goal_y=self.goal_y,
            number_of_rays=self.number_of_rays,
            n_sectors=self.n_sectors,
            range_min=self.range_min,
            range_max=self.range_max,
            maximum_goal_distance=self.maximum_goal_distance,
            angle_min=self.latest_scan.angle_min,
            angle_increment=self.latest_scan.angle_increment,
            front_half_angle_deg=self.front_half_angle_deg,
            include_front_clearance=self.include_front_clearance,
            front_normalization_distance=(
                self.front_normalization_distance),
            include_directional_clearance=(
                self.include_directional_clearance),
            directional_normalization_distance=(
                self.directional_normalization_distance),
            directional_inner_angle_deg=self.directional_inner_angle_deg,
            directional_outer_angle_deg=self.directional_outer_angle_deg,
            use_footprint_clearance=self.use_footprint_clearance,
            footprint_half_length=self.footprint_half_length,
            footprint_half_width=self.footprint_half_width,
            lidar_offset_x=self.lidar_offset_x,
            lidar_offset_y=self.lidar_offset_y,
            footprint_half_angle_deg=self.footprint_half_angle_deg,
        )

        if measurements['distance_to_goal'] <= self.goal_tolerance:
            self.finished = True
            self._publish_stop()
            self.get_logger().info(
                f'Goal reached in {self.episode_step} steps; '
                f'distance={measurements["distance_to_goal"]:.3f} m')
            return

        if self.episode_step >= self.maximum_episode_steps:
            self.finished = True
            self._publish_stop()
            self.get_logger().warning(
                f'Timeout after {self.episode_step} steps; '
                f'distance={measurements["distance_to_goal"]:.3f} m')
            return

        action, _ = self.model.predict(observation, deterministic=True)
        requested_linear, requested_angular = action_to_command(
            action,
            linear_speed_max=self.linear_speed_max,
            angular_speed_max=self.angular_speed_max,
            linear_reverse_speed_max=self.linear_reverse_speed_max,
        )
        linear_velocity, angular_velocity, shield_active = (
            apply_footprint_safety(
                requested_linear,
                requested_angular,
                measurements,
                enabled=self.enable_safety_shield,
                stop_distance=self.safety_stop_distance,
                slowdown_distance=self.safety_slowdown_distance,
                minimum_turn_speed=self.safety_minimum_turn_speed,
                angular_speed_max=self.angular_speed_max,
            )
        )
        command = Twist()
        command.linear.x = linear_velocity
        command.angular.z = angular_velocity
        self.command_publisher.publish(command)
        self.episode_step += 1

        self.get_logger().info(
            f'step={self.episode_step} '
            f'distance={measurements["distance_to_goal"]:.3f} '
            f'min_scan={measurements["minimum_scan"]:.3f} '
            f'front_scan={measurements["front_minimum_scan"]:.3f} '
            f'body_clearance={measurements["footprint_minimum_clearance"]:.3f} '
            f'escape_left={measurements["footprint_left_escape_clearance"]:.3f} '
            f'escape_right={measurements["footprint_right_escape_clearance"]:.3f} '
            f'raw_v={requested_linear:.3f} '
            f'shield={str(shield_active).lower()} '
            f'v={linear_velocity:.3f} w={angular_velocity:.3f}',
            throttle_duration_sec=1.0,
        )

    def _publish_goal_marker(self) -> None:
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'rl_goal'
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = self.goal_x
        marker.pose.position.y = self.goal_y
        marker.pose.position.z = 0.025
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.goal_tolerance * 2.0
        marker.scale.y = self.goal_tolerance * 2.0
        marker.scale.z = 0.05
        marker.color.r = 0.1
        marker.color.g = 1.0
        marker.color.b = 0.1
        marker.color.a = 0.9
        self.goal_publisher.publish(marker)

    def stop(self) -> None:
        self.finished = True
        if rclpy.ok():
            self._publish_stop()


def main() -> None:
    rclpy.init()
    node = None
    try:
        node = RosPolicyNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
