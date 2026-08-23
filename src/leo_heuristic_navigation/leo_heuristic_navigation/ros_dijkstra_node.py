"""Deploy Dijkstra planning and deterministic tracking through ROS 2."""

from __future__ import annotations

import math
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path as PathMessage
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker

from leo_heuristic_navigation.dijkstra_planner import PlanResult, plan_path
from leo_heuristic_navigation.grid_map import GridMap, load_grid_map
from leo_heuristic_navigation.path_controller import (
    ControllerConfig,
    PathController,
)


def _yaw_from_odometry(message: Odometry) -> float:
    quaternion = message.pose.pose.orientation
    sin_yaw = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cos_yaw = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    return math.atan2(sin_yaw, cos_yaw)


class DijkstraNavigationNode(Node):
    """Plan from the YAML map and track the path deterministically."""

    def __init__(self) -> None:
        super().__init__('leo_dijkstra_navigation_node')
        self.world_path = Path(
            self.declare_parameter('world_path', '').value
        ).expanduser().resolve()
        self.goal_x = float(self.declare_parameter('goal_x', 0.0).value)
        self.goal_y = float(self.declare_parameter('goal_y', 0.0).value)
        self.x_min = float(self.declare_parameter('arena_x_min', -5.0).value)
        self.x_max = float(self.declare_parameter('arena_x_max', 5.0).value)
        self.y_min = float(self.declare_parameter('arena_y_min', -5.0).value)
        self.y_max = float(self.declare_parameter('arena_y_max', 5.0).value)
        self.resolution = float(
            self.declare_parameter('grid_resolution', 0.08).value)
        self.inflation_radius = float(
            self.declare_parameter(
                'obstacle_inflation_radius', 0.42).value)
        self.allow_diagonal = bool(
            self.declare_parameter('allow_diagonal', True).value)
        self.goal_tolerance = float(
            self.declare_parameter('goal_tolerance', 0.25).value)
        self.maximum_episode_steps = int(
            self.declare_parameter('maximum_episode_steps', 900).value)
        self.control_hz = float(
            self.declare_parameter('control_hz', 10.0).value)
        self.sensor_timeout = float(
            self.declare_parameter('sensor_timeout', 0.50).value)
        self.emergency_replan_steps = int(
            self.declare_parameter('emergency_replan_steps', 10).value)

        self.controller_config = ControllerConfig(
            linear_speed_max=float(
                self.declare_parameter('linear_speed_max', 0.25).value),
            angular_speed_max=float(
                self.declare_parameter('angular_speed_max', 0.80).value),
            heading_gain=float(
                self.declare_parameter('heading_gain', 1.8).value),
            pivot_heading_threshold=float(
                self.declare_parameter(
                    'pivot_heading_threshold', 0.55).value),
            lookahead_distance=float(
                self.declare_parameter('lookahead_distance', 0.45).value),
            waypoint_tolerance=float(
                self.declare_parameter('waypoint_tolerance', 0.20).value),
            lidar_front_half_angle_deg=float(
                self.declare_parameter(
                    'lidar_front_half_angle_deg', 45.0).value),
            lidar_emergency_stop_distance=float(
                self.declare_parameter(
                    'lidar_emergency_stop_distance', 0.20).value),
            lidar_slowdown_distance=float(
                self.declare_parameter(
                    'lidar_slowdown_distance', 0.50).value),
        )
        if not self.world_path.is_file():
            raise FileNotFoundError(f'World does not exist: {self.world_path}')
        if self.control_hz <= 0.0 or self.sensor_timeout <= 0.0:
            raise ValueError('Control frequency and timeout must be positive')

        self.grid: GridMap = load_grid_map(
            self.world_path,
            x_min=self.x_min,
            x_max=self.x_max,
            y_min=self.y_min,
            y_max=self.y_max,
            resolution=self.resolution,
            inflation_radius=self.inflation_radius,
        )
        self.plan: PlanResult | None = None
        self.controller: PathController | None = None
        self.latest_scan: LaserScan | None = None
        self.latest_odometry: Odometry | None = None
        self.last_scan_time = None
        self.last_odometry_time = None
        self.collision = False
        self.finished = False
        self.episode_step = 0
        self.replan_count = 0
        self.emergency_steps = 0

        self.command_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        marker_qos = QoSProfile(depth=1)
        marker_qos.reliability = ReliabilityPolicy.RELIABLE
        marker_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.path_publisher = self.create_publisher(
            PathMessage, '/dijkstra_path', marker_qos)
        self.goal_publisher = self.create_publisher(
            Marker, '/dijkstra_goal', marker_qos)
        self.target_publisher = self.create_publisher(
            Marker, '/dijkstra_target', marker_qos)
        self.create_subscription(LaserScan, '/scan', self._scan_callback, 10)
        self.create_subscription(Odometry, '/odom', self._odom_callback, 10)
        self.create_subscription(
            Bool, '/collision', self._collision_callback, 10)
        self.timer = self.create_timer(1.0 / self.control_hz, self._control)
        self._publish_goal()
        self.get_logger().info(
            f'Dijkstra world={self.world_path}; '
            f'goal=({self.goal_x:.6f}, {self.goal_y:.6f}); '
            f'grid={self.grid.width}x{self.grid.height}; '
            f'resolution={self.resolution:.3f}; '
            f'inflation={self.inflation_radius:.3f}')

    def _scan_callback(self, message: LaserScan) -> None:
        self.latest_scan = message
        self.last_scan_time = self.get_clock().now()

    def _odom_callback(self, message: Odometry) -> None:
        self.latest_odometry = message
        self.last_odometry_time = self.get_clock().now()

    def _collision_callback(self, message: Bool) -> None:
        self.collision = bool(message.data)

    def _inputs_are_fresh(self) -> bool:
        if self.last_scan_time is None or self.last_odometry_time is None:
            return False
        timeout = Duration(seconds=self.sensor_timeout)
        now = self.get_clock().now()
        return (
            now - self.last_scan_time <= timeout and
            now - self.last_odometry_time <= timeout
        )

    def _publish_stop(self) -> None:
        self.command_publisher.publish(Twist())

    def _create_plan(self, x: float, y: float) -> None:
        self.plan = plan_path(
            self.grid,
            (x, y),
            (self.goal_x, self.goal_y),
            allow_diagonal=self.allow_diagonal,
        )
        self.controller = PathController(
            self.plan.path,
            self.controller_config,
        )
        self._publish_path()
        self.get_logger().info(
            f'Planned {len(self.plan.path)} waypoints; '
            f'cost={self.plan.cost_m:.3f} m; '
            f'expanded={self.plan.expanded_cells}; '
            f'time={self.plan.planning_time_s:.6f} s; '
            f'replans={self.replan_count}')

    def _publish_path(self) -> None:
        if self.plan is None:
            return
        message = PathMessage()
        message.header.frame_id = 'map'
        message.header.stamp = self.get_clock().now().to_msg()
        for x, y in self.plan.path:
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            message.poses.append(pose)
        self.path_publisher.publish(message)

    def _publish_goal(self) -> None:
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'dijkstra_goal'
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = self.goal_x
        marker.pose.position.y = self.goal_y
        marker.pose.position.z = 0.025
        marker.pose.orientation.w = 1.0
        marker.scale.x = 2.0 * self.goal_tolerance
        marker.scale.y = 2.0 * self.goal_tolerance
        marker.scale.z = 0.05
        marker.color.r = 0.1
        marker.color.g = 1.0
        marker.color.b = 0.1
        marker.color.a = 0.9
        self.goal_publisher.publish(marker)

    def _publish_target(self, target_index: int) -> None:
        if self.plan is None:
            return
        x, y = self.plan.path[target_index]
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'dijkstra_target'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.08
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.16
        marker.scale.y = 0.16
        marker.scale.z = 0.16
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.a = 0.9
        self.target_publisher.publish(marker)

    def _control(self) -> None:
        self._publish_goal()
        if self.finished:
            self._publish_stop()
            return
        if self.latest_scan is None or self.latest_odometry is None:
            self._publish_stop()
            return
        if not self._inputs_are_fresh():
            self._publish_stop()
            self.get_logger().warning(
                'Sensor data is stale; publishing zero velocity',
                throttle_duration_sec=2.0,
            )
            return
        if self.collision:
            self.finished = True
            self._publish_stop()
            self.get_logger().error(
                f'Collision at step {self.episode_step}; stopped')
            return

        position = self.latest_odometry.pose.pose.position
        yaw = _yaw_from_odometry(self.latest_odometry)
        distance = math.hypot(
            self.goal_x - position.x,
            self.goal_y - position.y,
        )
        if distance <= self.goal_tolerance:
            self.finished = True
            self._publish_stop()
            self.get_logger().info(
                f'Goal reached in {self.episode_step} steps; '
                f'distance={distance:.3f} m')
            return
        if self.episode_step >= self.maximum_episode_steps:
            self.finished = True
            self._publish_stop()
            self.get_logger().warning(
                f'Timeout after {self.episode_step} steps; '
                f'distance={distance:.3f} m')
            return
        try:
            if self.controller is None:
                self._create_plan(position.x, position.y)
            assert self.controller is not None
            command_result = self.controller.command(
                x=position.x,
                y=position.y,
                yaw=yaw,
                ranges=self.latest_scan.ranges,
                angle_min=self.latest_scan.angle_min,
                angle_increment=self.latest_scan.angle_increment,
                range_max=self.latest_scan.range_max,
            )
        except (RuntimeError, ValueError) as error:
            self.finished = True
            self._publish_stop()
            self.get_logger().error(f'Planning failed: {error}')
            return

        if command_result.emergency:
            self.emergency_steps += 1
        else:
            self.emergency_steps = 0
        if self.emergency_steps >= self.emergency_replan_steps:
            try:
                self.replan_count += 1
                self._create_plan(position.x, position.y)
                assert self.controller is not None
                command_result = self.controller.command(
                    x=position.x,
                    y=position.y,
                    yaw=yaw,
                    ranges=self.latest_scan.ranges,
                    angle_min=self.latest_scan.angle_min,
                    angle_increment=self.latest_scan.angle_increment,
                    range_max=self.latest_scan.range_max,
                )
                self.emergency_steps = int(command_result.emergency)
            except (RuntimeError, ValueError) as error:
                self.finished = True
                self._publish_stop()
                self.get_logger().error(f'Replanning failed: {error}')
                return

        command = Twist()
        command.linear.x = command_result.linear
        command.angular.z = command_result.angular
        self.command_publisher.publish(command)
        self._publish_target(command_result.target_index)
        self.episode_step += 1
        self.get_logger().info(
            f'step={self.episode_step} distance={distance:.3f} '
            f'target={command_result.target_index} '
            f'heading_error={command_result.heading_error:.3f} '
            f'front={command_result.front_clearance:.3f} '
            f'emergency={str(command_result.emergency).lower()} '
            f'v={command_result.linear:.3f} '
            f'w={command_result.angular:.3f}',
            throttle_duration_sec=1.0,
        )

    def stop(self) -> None:
        self.finished = True
        if rclpy.ok():
            self._publish_stop()


def main() -> None:
    """Run the ROS 2 Dijkstra navigation node."""
    rclpy.init()
    node = None
    try:
        node = DijkstraNavigationNode()
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
