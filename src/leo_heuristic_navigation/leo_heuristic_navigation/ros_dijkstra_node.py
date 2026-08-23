"""Deploy LiDAR-only mapping and online Dijkstra planning through ROS 2."""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path as PathMessage
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker

from leo_heuristic_navigation.online_navigation import (
    OnlineDijkstraNavigator,
)


def _yaw_from_odometry(message: Odometry) -> float:
    quaternion = message.pose.pose.orientation
    sin_yaw = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cos_yaw = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    return math.atan2(sin_yaw, cos_yaw)


class DijkstraNavigationNode(Node):
    """Navigate using only LiDAR, odometry, and the known goal position."""

    def __init__(self) -> None:
        super().__init__('leo_dijkstra_navigation_node')
        self.goal_x = float(self.declare_parameter('goal_x', 0.0).value)
        self.goal_y = float(self.declare_parameter('goal_y', 0.0).value)

        planner = {
            'arena_x_min': float(
                self.declare_parameter('arena_x_min', -5.0).value),
            'arena_x_max': float(
                self.declare_parameter('arena_x_max', 5.0).value),
            'arena_y_min': float(
                self.declare_parameter('arena_y_min', -5.0).value),
            'arena_y_max': float(
                self.declare_parameter('arena_y_max', 5.0).value),
            'grid_resolution': float(
                self.declare_parameter('grid_resolution', 0.08).value),
            'obstacle_inflation_radius': float(
                self.declare_parameter(
                    'obstacle_inflation_radius', 0.42).value),
            'allow_diagonal': bool(
                self.declare_parameter('allow_diagonal', True).value),
        }
        mapping = {
            'lidar_offset_x': float(
                self.declare_parameter('lidar_offset_x', 0.10).value),
            'lidar_offset_y': float(
                self.declare_parameter('lidar_offset_y', 0.0).value),
            'maximum_mapping_range': float(
                self.declare_parameter('maximum_mapping_range', 5.0).value),
            'ray_stride': int(
                self.declare_parameter('ray_stride', 2).value),
            'update_interval_steps': int(
                self.declare_parameter('update_interval_steps', 2).value),
            'periodic_replan_steps': int(
                self.declare_parameter('periodic_replan_steps', 20).value),
        }
        controller = {
            'control_hz': float(
                self.declare_parameter('control_hz', 10.0).value),
            'linear_speed_max': float(
                self.declare_parameter('linear_speed_max', 0.25).value),
            'angular_speed_max': float(
                self.declare_parameter('angular_speed_max', 0.80).value),
            'heading_gain': float(
                self.declare_parameter('heading_gain', 1.8).value),
            'pivot_heading_threshold': float(
                self.declare_parameter(
                    'pivot_heading_threshold', 0.55).value),
            'lookahead_distance': float(
                self.declare_parameter('lookahead_distance', 0.40).value),
            'waypoint_tolerance': float(
                self.declare_parameter('waypoint_tolerance', 0.20).value),
            'goal_tolerance': float(
                self.declare_parameter('goal_tolerance', 0.25).value),
            'lidar_front_half_angle_deg': float(
                self.declare_parameter(
                    'lidar_front_half_angle_deg', 55.0).value),
            'lidar_emergency_stop_distance': float(
                self.declare_parameter(
                    'lidar_emergency_stop_distance', 0.24).value),
            'lidar_slowdown_distance': float(
                self.declare_parameter(
                    'lidar_slowdown_distance', 0.65).value),
            'emergency_replan_steps': int(
                self.declare_parameter('emergency_replan_steps', 5).value),
            'sensor_timeout': float(
                self.declare_parameter('sensor_timeout', 0.50).value),
        }
        self.goal_tolerance = controller['goal_tolerance']
        self.control_hz = controller['control_hz']
        self.sensor_timeout = controller['sensor_timeout']
        self.maximum_episode_steps = int(
            self.declare_parameter('maximum_episode_steps', 900).value)
        if self.control_hz <= 0.0 or self.sensor_timeout <= 0.0:
            raise ValueError('Control frequency and timeout must be positive')

        self.navigator = OnlineDijkstraNavigator(
            planner=planner,
            mapping=mapping,
            controller=controller,
            goal_x=self.goal_x,
            goal_y=self.goal_y,
        )
        self.latest_scan: LaserScan | None = None
        self.latest_odometry: Odometry | None = None
        self.last_scan_time = None
        self.last_odometry_time = None
        self.collision = False
        self.finished = False
        self.episode_step = 0

        self.command_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        marker_qos = QoSProfile(depth=1)
        marker_qos.reliability = ReliabilityPolicy.RELIABLE
        marker_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.path_publisher = self.create_publisher(
            PathMessage, '/dijkstra_path', marker_qos)
        self.map_publisher = self.create_publisher(
            OccupancyGrid, '/dijkstra_discovered_map', marker_qos)
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
        self._publish_discovered_map()
        grid = self.navigator.mapper.grid
        self.get_logger().info(
            'Dijkstra obstacle_source=lidar_only; '
            f'goal=({self.goal_x:.6f}, {self.goal_y:.6f}); '
            f'grid={grid.width}x{grid.height}; '
            f'resolution={grid.resolution:.3f}; '
            f'initial_obstacle_cells={len(grid.occupied)}')

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

    def _publish_path(self) -> None:
        if self.navigator.plan is None:
            return
        message = PathMessage()
        message.header.frame_id = 'map'
        message.header.stamp = self.get_clock().now().to_msg()
        for x, y in self.navigator.plan.path:
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            message.poses.append(pose)
        self.path_publisher.publish(message)

    def _publish_discovered_map(self) -> None:
        mapper = self.navigator.mapper
        grid = mapper.grid
        message = OccupancyGrid()
        message.header.frame_id = 'map'
        message.header.stamp = self.get_clock().now().to_msg()
        message.info.resolution = grid.resolution
        message.info.width = grid.width
        message.info.height = grid.height
        message.info.origin.position.x = (
            grid.x_min - 0.5 * grid.resolution)
        message.info.origin.position.y = (
            grid.y_min - 0.5 * grid.resolution)
        message.info.origin.orientation.w = 1.0
        message.data = mapper.occupancy_values()
        self.map_publisher.publish(message)

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
        if self.navigator.plan is None:
            return
        x, y = self.navigator.plan.path[target_index]
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
                f'distance={distance:.3f} m; '
                f'replans={self.navigator.replans}; '
                f'discovered_obstacles='
                f'{len(self.navigator.mapper.obstacle_hits)}')
            return
        if self.episode_step >= self.maximum_episode_steps:
            self.finished = True
            self._publish_stop()
            self.get_logger().warning(
                f'Timeout after {self.episode_step} steps; '
                f'distance={distance:.3f} m')
            return

        try:
            update = self.navigator.update(
                robot_x=position.x,
                robot_y=position.y,
                robot_yaw=yaw,
                ranges=self.latest_scan.ranges,
                angle_min=self.latest_scan.angle_min,
                angle_increment=self.latest_scan.angle_increment,
                range_min=self.latest_scan.range_min,
                range_max=self.latest_scan.range_max,
            )
        except (RuntimeError, ValueError) as error:
            self.finished = True
            self._publish_stop()
            self.get_logger().error(f'Online planning failed: {error}')
            return

        if update.map_changed:
            self._publish_discovered_map()
        if update.plan_changed:
            self._publish_path()
            assert self.navigator.plan is not None
            self.get_logger().info(
                f'Online plan waypoints={len(self.navigator.plan.path)}; '
                f'cost={self.navigator.plan.cost_m:.3f} m; '
                f'replans={self.navigator.replans}; '
                f'discovered_obstacles='
                f'{len(self.navigator.mapper.obstacle_hits)}')

        command = Twist()
        command.linear.x = update.command.linear
        command.angular.z = update.command.angular
        self.command_publisher.publish(command)
        self._publish_target(update.command.target_index)
        self.episode_step += 1
        self.get_logger().info(
            f'step={self.episode_step} distance={distance:.3f} '
            f'target={update.command.target_index} '
            f'heading_error={update.command.heading_error:.3f} '
            f'front={update.command.front_clearance:.3f} '
            f'discovered={len(self.navigator.mapper.obstacle_hits)} '
            f'replans={self.navigator.replans} '
            f'emergency={str(update.command.emergency).lower()} '
            f'v={update.command.linear:.3f} '
            f'w={update.command.angular:.3f}',
            throttle_duration_sec=1.0,
        )

    def stop(self) -> None:
        self.finished = True
        if rclpy.ok():
            self._publish_stop()


def main() -> None:
    """Run the ROS 2 LiDAR-only Dijkstra navigation node."""
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
