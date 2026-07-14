#!/usr/bin/env python3
"""
Deploy the trained LeoRover2DEnv PPO policy on a real Leo Rover (ROS 2).

Runs ON-BOARD (Raspberry Pi). Rebuilds the *exact* observation the policy was
trained on from live sensors, runs PPO.predict at 10 Hz, and publishes /cmd_vel.

    /scan  (sensor_msgs/LaserScan)  -> 3200 normalized lidar ranges
    /odom  (nav_msgs/Odometry)      -> robot pose -> dist + bearing to goal
    goal   (ROS params goal_x,goal_y, in the /odom frame)

    -> action [v, omega] -> geometry_msgs/Twist on /cmd_vel

Run:
    # source your ROS 2 + the venv that has stable-baselines3 / torch
    python3 leo_deploy.py --ros-args -p goal_x:=2.0 -p goal_y:=1.0

IMPORTANT sim2real notes are at the bottom of this file. The policy in
leo_ppo.zip was trained with lidar_noise=0.0 and ideal kinematics -- retrain
with domain randomization before trusting it at speed.
"""

import math
import numpy as np
from gymnasium import spaces

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from enum import Enum

from sympy.strategies.core import switch


class EState(Enum):
    STOPPED = 0,
    REACHED = 1,
    CHASE_GOAL = 2,
    AVOIDING_OBSTACLE = 3

class EObstacleState(Enum):
    ROTATE = 0,
    FOLLOW = 1


# --- NumPy 1.x / 2.x pickle compatibility ------------------------------------
# A model saved under NumPy 2.x pickles references to the internal module
# 'numpy._core', which does not exist on NumPy 1.x (it was 'numpy.core'). If
# this Jetson still ships NumPy 1.x, alias the names so the policy unpickles.
# (Upgrading the Jetson's NumPy instead risks breaking rclpy / ROS packages
# compiled against the NumPy 1.x ABI, so we shim rather than upgrade.)
import sys as _sys
if not hasattr(np, "_core"):
    import numpy.core as _np_core
    _sys.modules["numpy._core"] = _np_core
    for _sub in ("multiarray", "numeric", "umath", "_multiarray_umath", "overrides"):
        try:
            _sys.modules[f"numpy._core.{_sub}"] = __import__(
                f"numpy.core.{_sub}", fromlist=[_sub]
            )
        except Exception:
            pass

# --- constants that MUST match training (see leo_rover_env.py) ---------------
N_BEAMS = 3200
# Resampled-beam indices for alignment checks. beam_rel = linspace(-pi, pi, N,
# endpoint=False), so 0 rad (robot forward) is at N/2, +pi/2 (left) at 3N/4,
# -pi/2 (right) at N/4.
FWD_BEAM = N_BEAMS // 2                        # 1600 -> straight ahead
LEFT_BEAM = 3 * N_BEAMS // 4                   # 2400 -> rover's left (CCW)
RIGHT_BEAM = N_BEAMS // 4                       # 800  -> rover's right
LIDAR_FOV = 2 * math.pi                       # 360 deg scan
LIDAR_MAX_RANGE = 10.0                          # meters
V_MAX = 0.4                                    # m/s
W_MAX = 1.0                                   # rad/s
GOAL_TOL = 0.25                                # m
ROBOT_RADIUS = 0.18                            # m
# Distance was normalized by the ARENA diagonal in sim, NOT the real room.
# Keep the training constant or the input goes out of distribution.
DIAG = math.hypot(6.0, 6.0)                    # sqrt(72) ~= 8.485

CONTROL_HZ = 10.0                              # dt = 0.1 in sim
SENSOR_TIMEOUT = 0.5                           # s; stop if data goes stale
SAFETY_MARGIN = 0.10                           # m; hard-stop clearance


def yaw_from_quat(q):
    """Yaw (Z) from a geometry_msgs quaternion."""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class BugDeploy(Node):
    def __init__(self):
        super().__init__("leo_deploy")

        # --- parameters ---
        self.declare_parameter("goal_x", 2.0)
        self.declare_parameter("goal_y", 0.0)
        self.declare_parameter("model_path", "leo_ppo")
        # Mounting yaw of the lidar frame relative to base_link (rad). If the
        # lidar's 0 deg does not point straight forward, set this. CCW positive.
        self.declare_parameter("lidar_yaw_offset", 0.0)
        # Topic names. Defaults follow the generic ROS conventions; override for
        # this rover, e.g.
        #   -p odom_topic:=/merged_odom -p cmd_vel_topic:=/rob_1/cmd_vel
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        # When true, log the resampled forward/left/right beams each step so you
        # can verify lidar_yaw_offset and spin handedness before driving.
        self.declare_parameter("debug", False)
        self.debug = bool(self.get_parameter("debug").value)

        self.goal = np.array([
            self.get_parameter("goal_x").value,
            self.get_parameter("goal_y").value,
        ], dtype=np.float32)
        self.lidar_yaw_offset = float(self.get_parameter("lidar_yaw_offset").value)

        # Sim beam angles (robot-relative). 360 deg, evenly spaced, CCW, 0=forward.
        self._beam_rel = np.linspace(-math.pi, math.pi, N_BEAMS, endpoint=False)

        # --- state filled by callbacks ---
        self._scan = None          # latest resampled ranges (meters), len N_BEAMS
        self._scan_stamp = None
        self._pose = None          # (x, y, yaw)
        self._pose_stamp = None

        # --- ROS plumbing ---
        scan_topic = self.get_parameter("scan_topic").value
        odom_topic = self.get_parameter("odom_topic").value
        cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.get_logger().info(
            f"topics: scan={scan_topic} odom={odom_topic} cmd_vel={cmd_vel_topic}"
        )
        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.create_subscription(LaserScan, scan_topic, self._on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, odom_topic, self._on_odom, 10)
        self.create_timer(1.0 / CONTROL_HZ, self._control_step)

        self.state = EState.CHASE_GOAL
        self.obstacle_state = EObstacleState.ROTATE

    # ---------------------------------------------------------------- sensors
    def _on_scan(self, msg: LaserScan):
        """Resample the real scan onto the 3200 training beam angles."""
        n = len(msg.ranges)
        if n == 0:
            return
        raw = np.asarray(msg.ranges, dtype=np.float64)

        # Map each target (robot-frame) angle to the nearest real beam index.
        # base_angle = lidar_angle + offset  =>  lidar_angle = base_angle - offset
        lidar_ang = self._beam_rel - self.lidar_yaw_offset
        idx = np.round((lidar_ang - msg.angle_min) / msg.angle_increment).astype(int)
        idx = np.mod(idx, n)                       # wrap around the full scan
        ranges = raw[idx]

        # Classify returns. CRITICAL: a reading below range_min is NOT free space
        # -- it is a real obstacle closer than the sensor's spec'd minimum (the
        # C1 reports ~0.10 m for things right against the rover). Treating those
        # as max range would blind the safety stop to the closest obstacles.
        #   no return (inf/nan/0.0/beyond range_max) -> free space (max range)
        #   small positive below range_min           -> very-close obstacle; floor
        #                                                at range_min so min() stays
        #                                                small and the stop fires.
        no_return = ~np.isfinite(ranges) | (ranges <= 0.0) | (ranges > msg.range_max)
        too_close = np.isfinite(ranges) & (ranges > 0.0) & (ranges < msg.range_min)
        ranges[no_return] = LIDAR_MAX_RANGE
        ranges[too_close] = msg.range_min
        ranges = np.clip(ranges, 0.0, LIDAR_MAX_RANGE)

        self._scan = ranges
        self._scan_stamp = self.get_clock().now()

    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        yaw = yaw_from_quat(msg.pose.pose.orientation)
        self._pose = (p.x, p.y, yaw)
        self._pose_stamp = self.get_clock().now()

    # ---------------------------------------------------------------- obs
    def _build_obs(self):
        norm_ranges = (self._scan / LIDAR_MAX_RANGE).astype(np.float32)
        x, y, yaw = self._pose
        dx = self.goal[0] - x
        dy = self.goal[1] - y
        dist = math.hypot(dx, dy)
        dist_norm = min(dist / DIAG, 1.0)
        bearing = wrap(math.atan2(dy, dx) - yaw)
        obs = np.concatenate(
            [norm_ranges, [dist_norm], [math.sin(bearing), math.cos(bearing)]]
        ).astype(np.float32)
        return obs, dist

    # ---------------------------------------------------------------- loop
    def _stale(self, stamp):
        if stamp is None:
            return True
        dt = (self.get_clock().now() - stamp).nanoseconds * 1e-9
        return dt > SENSOR_TIMEOUT

    def _stop(self, reason=None):
        self.cmd_pub.publish(Twist())   # all-zero
        if reason:
            self.get_logger().warn(f"STOP: {reason}", throttle_duration_sec=1.0)

    def _control_step(self):
        # Alignment check runs as soon as /scan exists, independent of odom, so
        # you can set lidar_yaw_offset before odom is even wired up. Put an
        # object dead ahead -> fwd should be the small one; move it to the
        # rover's LEFT -> left should drop (confirms CCW handedness).
        if self.debug and self._scan is not None:
            self.get_logger().info(
                f"fwd={self._scan[FWD_BEAM]:.2f}  "
                f"left={self._scan[LEFT_BEAM]:.2f}  "
                f"right={self._scan[RIGHT_BEAM]:.2f}  "
                f"min={float(self._scan.min()):.2f}",
                throttle_duration_sec=0.5,
            )

        if self.state == EState.REACHED:
            self._stop()
            return

        # Watchdog: never drive on stale / missing sensor data.
        if self._scan is None or self._pose is None:
            missing = [n for n, v in (("scan", self._scan), ("odom", self._pose)) if v is None]
            self._stop(f"waiting for: {', '.join(missing)} (no messages received yet)")
            return
        if self._stale(self._scan_stamp) or self._stale(self._pose_stamp):
            stale = [n for n, s in (("scan", self._scan_stamp), ("odom", self._pose_stamp))
                     if self._stale(s)]
            self._stop(f"sensor data stale: {', '.join(stale)}")
            return

        # Hard safety stop: real obstacle inside the footprint + margin.
        if float(self._scan.min()) < ROBOT_RADIUS + SAFETY_MARGIN:
            self._stop("obstacle within safety margin")
            return

        obs, dist = self._build_obs()
        x1, y1, yaw = self._pose
        xt, yt = self.goal
        x2, y2 = (np.cos(yaw), np.sin(yaw))  # (1, 0) rotated by yaw

        vec1 = np.array((xt - x1, yt - y1))
        vec1 = vec1 / np.linalg.norm(vec1)
        vec2 = np.array((x2, y2))
        vec2 = vec2 / np.linalg.norm(vec2)
        cosine_distance = np.dot(vec1, vec2)
        angle_diff = np.arccos(cosine_distance)

        # State transitions
        if dist < GOAL_TOL:
            # If we're close, stop
            self.state = EState.REACHED
            self._stop()
            self.get_logger().info(f"goal reached (dist={dist:.2f} m)")
            return

        if self.state == EState.CHASE_GOAL and self._scan[FWD_BEAM] < ROBOT_RADIUS + 2*SAFETY_MARGIN:
            # There's an obstacle in the way. Bug around it
            self.get_logger().info(f"Change of state: Now avoiding obstacle")
            self.state = EState.AVOIDING_OBSTACLE
            self.obstacle_state = EObstacleState.ROTATE
        elif self.state == EState.AVOIDING_OBSTACLE and self._scan[FWD_BEAM] > ROBOT_RADIUS + 2*SAFETY_MARGIN and angle_diff < 0.1*np.pi:
            # The path forward is clear and we're pointing towards the goal. Get going
            self.get_logger().info(f"Change of state: Now driving towards goal")
            self.state = EState.CHASE_GOAL

        # Action time
        cmd = Twist()

        if self.state == EState.CHASE_GOAL:
            # Go as fast as possible and rotate towards goal
            cmd.linear.x = V_MAX

            # Determine direction and angular velocity
            d = (xt - x1)*(y2-y1)-(yt-y1)*(x2-x1)
            left_reference = ((x1 - 1) - x1)*(y2-y1)-(yt-y1)*(x2-x1)  # Guaranteed to be 'to the left'
            left_right = np.sign(d) * np.sign(left_reference)  # 1 == should turn left, -1 == should turn right

            vec1 = np.array((xt - x1, yt - y1))
            vec1 = vec1 / np.linalg.norm(vec1)
            vec2 = np.array((x2, y2))
            vec2 = vec2 / np.linalg.norm(vec2)
            cosine_distance = np.dot(vec1, vec2)

            cmd.angular.z = left_right * cosine_distance * W_MAX

        if self.state == EState.AVOIDING_OBSTACLE:
            # Substate transitions
            distance_to_wall = self._scan[RIGHT_BEAM] - (ROBOT_RADIUS + 2 * SAFETY_MARGIN)
            if self.obstacle_state == EObstacleState.ROTATE and np.abs(distance_to_wall) < SAFETY_MARGIN:
                # Follow the wall to the right if it's close enough
                self.get_logger().info(f"Change of substate: Following obstacle")
                self.obstacle_state = EObstacleState.FOLLOW
            if self._scan[FWD_BEAM] < ROBOT_RADIUS + 2*SAFETY_MARGIN:
                # Make sure there's enough space ahead
                self.obstacle_state = EObstacleState.ROTATE
                self.get_logger().info(f"Change of substate: Rotating until obstacle is to the right")


            # Substate actions
            if self.obstacle_state == EObstacleState.ROTATE:
                # Just rotate
                cmd.linear.x = 0
                cmd.angular.z = W_MAX
            elif self.obstacle_state == EObstacleState.FOLLOW:
                # Go forward slowly, but rotate to make sure the wall to the right is roughly the distance at all times
                cmd.linear.x = 0.1 * V_MAX

                rotation_factor = distance_to_wall / SAFETY_MARGIN  # Negative == Too close. Positive == Too far away
                cmd.angular.z = -1 * 0.2 * rotation_factor * W_MAX

        self.get_logger().info(f"Publishing Twist command with linear: {cmd.linear.x}, angular: {cmd.angular.z}")
        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = BugDeploy()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node._stop()        # leave the rover stationary
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()


# ---------------------------------------------------------------------------
# Sim2real checklist before you trust this on hardware
# ---------------------------------------------------------------------------
# 1. LIDAR ALIGNMENT is the #1 cause of "it turns the wrong way." Verify that
#    beam index 1600 (relative angle 0) points straight FORWARD on the rover and
#    that your lidar scans CCW. If 0 deg isn't forward, set -p lidar_yaw_offset.
#    Sanity check: put an obstacle dead ahead and confirm self._scan[1600] is small.
#
# 2. The policy is now trained with lidar_noise=0.5 (heavy multiplicative beam
#    noise), but still uses perfect circular obstacles, ideal unicycle
#    kinematics, and zero latency. For a smaller sim2real gap also jitter
#    v_max/w_max/dt and add one step of action delay. Note 0.5 is aggressive --
#    if the policy won't converge, drop toward the 0.02-0.05 range.
#
# 3. /odom drifts. For a fixed goal a few meters away this is fine; for longer
#    runs switch to a map->base_link TF (SLAM/AMCL) and read the goal in /map.
#
# 4. DIAG and LIDAR_MAX_RANGE here MUST equal the training values, not your real
#    room. Changing them silently shifts the policy's inputs out of distribution.
#
# 5. First runs: tether the rover, keep V_MAX low, and have a kill switch on
#    /cmd_vel ready.
