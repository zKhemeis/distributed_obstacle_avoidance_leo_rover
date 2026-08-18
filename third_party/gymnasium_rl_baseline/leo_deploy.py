#!/usr/bin/env python3
"""
Deploy the trained LeoRover2DEnv PPO policy on a real Leo Rover (ROS 2).

Runs ON-BOARD (Raspberry Pi). Rebuilds the *exact* observation the policy was
trained on from live sensors, runs the forward-only PPO policy at 10 Hz, and
publishes /cmd_vel. (The RecurrentPPO/LSTM avenue was abandoned.)

    /scan  (sensor_msgs/LaserScan)  -> 500 normalized lidar ranges
    /odom  (nav_msgs/Odometry)      -> robot pose -> dist + bearing to goal
    goal   (ROS params goal_x,goal_y, in the /odom frame)

    -> action [v, omega] -> geometry_msgs/Twist on /cmd_vel

Run:
    # source your ROS 2 + the venv that has stable-baselines3 / torch
    python3 leo_deploy.py --ros-args -p goal_x:=2.0 -p goal_y:=1.0

IMPORTANT sim2real notes are at the bottom of this file. Retrain (see train.py,
which now enables domain randomization) whenever the sim assumptions change, and
keep the first hardware runs tethered and slow.
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

from stable_baselines3 import PPO
# from sb3_contrib import RecurrentPPO   # RecurrentPPO/LSTM avenue abandoned

# The deploy policy is forward-only plain PPO (best: 125221). The RecurrentPPO
# auto-detect below is retired now that the LSTM avenue is dropped:
# import zipfile
# def algo_for(model_path):
#     path = model_path if model_path.endswith(".zip") else model_path + ".zip"
#     with zipfile.ZipFile(path) as z:
#         blob = z.read("data")
#     return RecurrentPPO if b"recurrent" in blob.lower() else PPO

# --- constants that MUST match training (see leo_rover_env.py) ---------------
N_BEAMS = 500
# The policy OBSERVATION is the 500 resampled beams pooled into N_SECTORS
# sector-min values (min range per 7.2 deg wedge) -- MUST match n_sectors in
# train.py / leo_rover_env.py. The safety governor and the debug beam printout
# still use the full 500-beam resolution; only _build_obs pools.
N_SECTORS = 50
# Observation history, matching training (obs_stack / obs_last_action in
# train.py): N_STACK frames (oldest first), each [sectors, dist, sin, cos],
# followed by the previously commanded [v, w] (the policy's own last output,
# before sign flips and the safety governor -- that is what training stored).
N_STACK = 3
# Resampled-beam indices for alignment checks. beam_rel = linspace(-pi, pi, N,
# endpoint=False), so 0 rad (robot forward) is at N/2, +pi/2 (left) at 3N/4,
# -pi/2 (right) at N/4.
FWD_BEAM = N_BEAMS // 2                        # 250 -> straight ahead
LEFT_BEAM = 3 * N_BEAMS // 4                   # 375 -> rover's left (CCW)
RIGHT_BEAM = N_BEAMS // 4                       # 125 -> rover's right
LIDAR_FOV = 2 * math.pi                       # 360 deg scan
LIDAR_MAX_RANGE = 10.0                          # meters
V_MAX = 0.4                                    # m/s
W_MAX = 1.0                                   # rad/s
GOAL_TOL = 0.25                                # m
# Rectangular footprint from the lidar origin (front, back, side), meters. MUST
# match leo_rover_env.py's `footprint`. The lidar sits toward the front, so the
# body extends further back. Used by the safety governor to know the true edge
# distance in each direction (tight front, wide back).
FOOT_FRONT = 0.17
FOOT_BACK = 0.30
FOOT_SIDE = 0.25
# Distance was normalized by the ARENA diagonal in sim, NOT the real room.
# MUST equal leo_rover_env.py's arena diagonal or the input goes out of
# distribution. Arena is now (3.0, 2.0) -> hypot(3, 2) = sqrt(13) ~= 3.606.
DIAG = math.hypot(3.0, 2.0)                    # sqrt(13) ~= 3.606

CONTROL_HZ = 10.0                              # dt = 0.1 in sim
SENSOR_TIMEOUT = 0.5                           # s; stop if data goes stale
# --- directional safety governor -------------------------------------------
# Instead of a single symmetric hard-stop (which made the rover FREEZE whenever
# anything came within ~0.28 m, e.g. an obstacle passing alongside), gate only
# the *velocity direction* that drives toward a near obstacle, and leave turning
# free so the rover can spin toward an opening and escape instead of locking up.
#
# Thresholds follow the true rectangular footprint: for each beam we precompute
# the distance from the lidar to the footprint EDGE in that direction, and stop
# that direction of travel once an obstacle comes within edge + SAFETY_GAP. This
# is naturally tight in front (0.17 m) and wide in back (0.30 m).
SAFETY_GAP = 0.06                              # m clearance to keep beyond the body edge
FRONT_HALF = math.radians(60)                  # half-angle of the front/rear cones
HARD_GAP = 0.03                                # m; obstacle within edge+this -> also cut w
# Anything the lidar reports closer than this is treated as the rover's OWN body
# (e.g. the Jetson mounted on top at ~0.15 m) and ignored -> free space. It MUST
# be < the front footprint (0.17 m): if it were >=, it would mask real obstacles
# right in front of the bumper and the front safety-stop could never fire. So it
# sits just above the ~0.15 m self-return. Validate on hardware; tune via
# -p self_clearance:= if a phantom front obstacle appears or a real one is missed.
SELF_CLEARANCE = 0.16                          # m
# The Jetson behind the lidar occludes a fixed rear sector. Force those beams to
# max range so the observation matches training (see leo_rover_env.py blind_*).
# MUST equal the training values. Reverse is allowed, but the rear stays blind,
# so the policy was trained to reverse cautiously (collision penalties there).
BLIND_CENTER = math.pi                         # rover-relative sector center (pi = rear)
BLIND_WIDTH = math.radians(90)                 # occluded sector width


def yaw_from_quat(q):
    """Yaw (Z) from a geometry_msgs quaternion."""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class LeoDeploy(Node):
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
        # Odometry message type:
        #   "nav" -> nav_msgs/Odometry (pose from quaternion)  [default]
        #   "leo" -> leo_msgs/WheelOdom (raw firmware dead-reckoning; pose_x/y/yaw
        #            already relative to the firmware start pose). Use this to
        #            bypass a broken nav_msgs odom pipeline and read the firmware
        #            directly, e.g. -p odom_type:=leo -p odom_topic:=/firmware/wheel_odom
        self.declare_parameter("odom_type", "nav")
        # Skid-steer wheel odometry OVERCOUNTS chassis rotation: it measures the
        # wheels, which scrub sideways when pivoting, so it reports more yaw than
        # the body actually turned. leo_calibrate.py measures the factor (scan-
        # matching gives true chassis yaw): jetson-03 read ~1.84x. Correct it here
        # by scaling the read yaw by 1/1.84 ~= 0.543 so the bearing-to-goal the
        # policy sees agrees with the (true-frame) lidar -- otherwise the policy
        # sees the goal swing past while obstacles barely move, and spins. This is
        # a systematic per-rover bias, NOT randomization; measure it per robot.
        #   jetson-03: -p odom_yaw_scale:=0.543
        self.declare_parameter("odom_yaw_scale", 1.0)
        self.odom_yaw_scale = float(self.get_parameter("odom_yaw_scale").value)
        # When true, log the resampled forward/left/right beams each step so you
        # can verify lidar_yaw_offset and spin handedness before driving.
        self.declare_parameter("debug", False)
        self.debug = bool(self.get_parameter("debug").value)
        # Ignore lidar returns closer than this (m). Covers the Jetson mounted on
        # top (~0.15 m). See SELF_CLEARANCE. Override with -p self_clearance:=0.22
        self.declare_parameter("self_clearance", SELF_CLEARANCE)
        self.self_clearance = float(self.get_parameter("self_clearance").value)
        # Convention fixes you can apply WITHOUT retraining, once the self-test
        # below tells you which way the rover actually moves:
        #   invert_v -> +linear.x drives backward on this rover; flip it.
        #   invert_w -> +angular.z turns clockwise; flip it.
        self.declare_parameter("invert_v", False)
        self.declare_parameter("invert_w", False)
        self.v_sign = -1.0 if bool(self.get_parameter("invert_v").value) else 1.0
        self.w_sign = -1.0 if bool(self.get_parameter("invert_w").value) else 1.0
        # Open-loop motion self-test: drive a scripted forward + left-turn and
        # log the odom delta, so you can confirm "forward is forward" and that
        # +angular.z is CCW BEFORE trusting the policy. Runs instead of the policy.
        self.declare_parameter("selftest", False)
        self.selftest = bool(self.get_parameter("selftest").value)
        self._selftest_t0 = None
        self._selftest_p0 = None

        self.goal = np.array([
            self.get_parameter("goal_x").value,
            self.get_parameter("goal_y").value,
        ], dtype=np.float32)
        self.lidar_yaw_offset = float(self.get_parameter("lidar_yaw_offset").value)

        model_path = self.get_parameter("model_path").value
        # Rebuild the obs/action spaces locally instead of unpickling the ones
        # saved in the zip. If training ran on NumPy 2.x and this Jetson has
        # NumPy 1.x, unpickling the saved spaces fails ("No module named
        # 'numpy._core'"). These must match leo_rover_env.py exactly.
        frame_low = np.concatenate([np.zeros(N_SECTORS), [0.0], [-1.0, -1.0]])
        frame_high = np.concatenate([np.ones(N_SECTORS), [1.0], [1.0, 1.0]])
        obs_low = np.concatenate([np.tile(frame_low, N_STACK), [-1.0, -1.0]]).astype(np.float32)
        obs_high = np.concatenate([np.tile(frame_high, N_STACK), [1.0, 1.0]]).astype(np.float32)
        # Action space v in [0,1], omega in [-1,1] -- must match leo_rover_env.py
        # (allow_reverse=False). The policy is forward-only: reverse was removed
        # because it was the mechanism behind the goal-approach crab-walk. Turning
        # in place (v=0) still lets it pivot toward an opening. The rear-cone
        # branch of the governor below is now effectively dead (v is never < 0)
        # but is left in as a guard.
        custom_objects = {
            "observation_space": spaces.Box(obs_low, obs_high, dtype=np.float32),
            "action_space": spaces.Box(
                low=np.array([0.0, -1.0], dtype=np.float32),
                high=np.array([1.0, 1.0], dtype=np.float32),
            ),
            "lr_schedule": lambda _: 0.0,
            "clip_range": lambda _: 0.0,
        }
        # The deploy policy is forward-only plain PPO (best: 125221). The
        # RecurrentPPO/LSTM avenue is retired, so load PPO directly.
        self.model = PPO.load(model_path, device="cpu",
                              custom_objects=custom_objects)
        # RecurrentPPO hidden-state fields (unused; PPO is memoryless):
        # self._lstm_state = None
        # self._ep_start = True
        self.get_logger().info(f"loaded policy: {model_path}  goal={self.goal.tolist()}")

        # Sim beam angles (robot-relative). 360 deg, evenly spaced, CCW, 0=forward.
        self._beam_rel = np.linspace(-math.pi, math.pi, N_BEAMS, endpoint=False)
        # Beams occluded by the Jetson (fixed rover-relative rear sector).
        _bd = np.arctan2(np.sin(self._beam_rel - BLIND_CENTER),
                         np.cos(self._beam_rel - BLIND_CENTER))
        self._blind_mask = np.abs(_bd) <= BLIND_WIDTH / 2.0

        # Directional safety governor precompute (see the constants above):
        #   front cone -> gates forward drive (v > 0)
        #   rear  cone -> gates reverse       (v < 0)
        self._front_mask = np.abs(self._beam_rel) <= FRONT_HALF
        _rear = np.abs(np.arctan2(np.sin(self._beam_rel - math.pi),
                                  np.cos(self._beam_rel - math.pi)))
        self._rear_mask = _rear <= FRONT_HALF
        # Per-beam distance from the lidar to the footprint rectangle edge along
        # that beam (rectangle x in [-FOOT_BACK, FOOT_FRONT], y in [-FOOT_SIDE,
        # FOOT_SIDE]). The ray from the origin (inside the box) exits at the
        # nearest axis boundary. This gives a direction-aware body size.
        _c, _s = np.cos(self._beam_rel), np.sin(self._beam_rel)
        _eps = 1e-9
        _tx = np.where(_c > _eps, FOOT_FRONT / np.maximum(_c, _eps),
                       np.where(_c < -_eps, FOOT_BACK / np.maximum(-_c, _eps), np.inf))
        _ty = np.where(_s > _eps, FOOT_SIDE / np.maximum(_s, _eps),
                       np.where(_s < -_eps, FOOT_SIDE / np.maximum(-_s, _eps), np.inf))
        self._edge = np.minimum(_tx, _ty)          # center -> body edge per beam
        self._stop_range = self._edge + SAFETY_GAP  # block travel within this range
        self._hard_range = self._edge + HARD_GAP    # full stop within this range

        # --- state filled by callbacks ---
        self._scan = None          # latest resampled ranges (meters), len N_BEAMS
        self._scan_stamp = None
        self._pose = None          # (x, y, yaw)
        self._pose_stamp = None
        self._reached = False
        self._frames = None        # rolling frame stack (seeded on first obs)
        self._last_action = np.zeros(2, dtype=np.float32)

        # --- ROS plumbing ---
        scan_topic = self.get_parameter("scan_topic").value
        odom_topic = self.get_parameter("odom_topic").value
        cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        odom_type = self.get_parameter("odom_type").value
        self.get_logger().info(
            f"topics: scan={scan_topic} odom={odom_topic} ({odom_type}) "
            f"cmd_vel={cmd_vel_topic}"
        )
        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.create_subscription(LaserScan, scan_topic, self._on_scan, qos_profile_sensor_data)
        if odom_type == "leo":
            # Imported lazily so "nav" mode never requires leo_msgs on the host.
            from leo_msgs.msg import WheelOdom
            # best-effort sub is compatible with either a reliable or best-effort
            # firmware publisher (a reliable sub would silently drop a best-effort one).
            self.create_subscription(WheelOdom, odom_topic, self._on_leo_odom,
                                     qos_profile_sensor_data)
        else:
            self.create_subscription(Odometry, odom_topic, self._on_odom, 10)
        self.create_timer(1.0 / CONTROL_HZ, self._control_step)

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

        # Classify returns.
        #   no return (inf/nan/0.0/beyond range_max) -> free space (max range)
        #   closer than self_clearance               -> the rover's OWN body (the
        #     (default 0.20 m, covers the Jetson         Jetson blocks part of the
        #      at ~0.15 m)                               scan); treat as free space
        #                                                so it neither trips the
        #                                                safety stop nor feeds the
        #                                                policy a phantom obstacle.
        #   otherwise                                -> keep as measured.
        # NOTE: this is a global distance gate, so a genuine obstacle closer than
        # self_clearance is also ignored. That is acceptable here (the footprint is
        # 0.18 m, so anything inside 0.20 m is already touching), and is exactly
        # what lets the fixed Jetson return be filtered out. If you know the Jetson
        # only occupies a fixed angular sector, gate by beam angle instead to keep
        # full close-range sensitivity elsewhere.
        finite = np.isfinite(ranges)
        no_return = ~finite | (ranges <= 0.0) | (ranges > msg.range_max)
        self_return = finite & (ranges > 0.0) & (ranges < self.self_clearance)
        ranges[no_return] = LIDAR_MAX_RANGE
        ranges[self_return] = LIDAR_MAX_RANGE
        # Jetson-occluded rear sector -> max range, matching training exactly.
        ranges[self._blind_mask] = LIDAR_MAX_RANGE
        ranges = np.clip(ranges, 0.0, LIDAR_MAX_RANGE)

        self._scan = ranges
        self._scan_stamp = self.get_clock().now()

    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        yaw = yaw_from_quat(msg.pose.pose.orientation) * self.odom_yaw_scale
        self._pose = (p.x, p.y, yaw)
        self._pose_stamp = self.get_clock().now()

    def _on_leo_odom(self, msg):
        """leo_msgs/WheelOdom: pose_* is already x/y/yaw vs. the firmware start
        pose, so no quaternion conversion is needed. Same frame convention as the
        nav_msgs path (+x forward, +y left, yaw CCW); /reset_odometry re-zeros it.
        yaw is scaled by odom_yaw_scale to undo skid-steer wheel-odom overcount."""
        self._pose = (msg.pose_x, msg.pose_y, msg.pose_yaw * self.odom_yaw_scale)
        self._pose_stamp = self.get_clock().now()

    # ---------------------------------------------------------------- obs
    def _build_obs(self):
        # Sector-min pooling, exactly as in training (leo_rover_env._get_frame):
        # consecutive resampled beams are consecutive angles, so a reshape
        # groups each 7.2 deg wedge; min = nearest return in that wedge. The
        # self-clearance and blind-sector masking already happened in _on_scan.
        sectors = self._scan.reshape(N_SECTORS, -1).min(axis=1)
        norm_ranges = (sectors / LIDAR_MAX_RANGE).astype(np.float32)
        x, y, yaw = self._pose
        dx = self.goal[0] - x
        dy = self.goal[1] - y
        dist = math.hypot(dx, dy)
        dist_norm = min(dist / DIAG, 1.0)
        bearing = wrap(math.atan2(dy, dx) - yaw)
        frame = np.concatenate(
            [norm_ranges, [dist_norm], [math.sin(bearing), math.cos(bearing)]]
        ).astype(np.float32)
        # Frame stack (oldest first) + last commanded action, as in training.
        if self._frames is None:
            self._frames = [frame] * N_STACK
        else:
            self._frames.pop(0)
            self._frames.append(frame)
        obs = np.concatenate(self._frames + [self._last_action]).astype(np.float32)
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
            imin = int(np.argmin(self._scan))
            min_ang = math.degrees(self._beam_rel[imin])
            self.get_logger().info(
                f"fwd={self._scan[FWD_BEAM]:.2f}  "
                f"left={self._scan[LEFT_BEAM]:.2f}  "
                f"right={self._scan[RIGHT_BEAM]:.2f}  "
                f"min={float(self._scan.min()):.2f} @ {min_ang:+.0f}deg",
                throttle_duration_sec=0.5,
            )

        # Open-loop motion self-test (Issue: "is forward actually forward?").
        # Runs a scripted forward drive then a left (CCW) turn and reports the
        # odom delta, so you can confirm the drive + turn conventions independently
        # of the policy. Enable with -p selftest:=true.
        if self.selftest:
            self._run_selftest()
            return

        if self._reached:
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

        obs, dist = self._build_obs()

        if dist < GOAL_TOL:
            self._reached = True
            self._stop()
            self.get_logger().info(f"goal reached (dist={dist:.2f} m)")
            return

        # PPO is memoryless; predict per control step. (RecurrentPPO LSTM-state
        # threading removed with the LSTM avenue.)
        action, _ = self.model.predict(obs, deterministic=True)
        # v in [0,1], omega in [-1,1] (forward-only policy, allow_reverse=False).
        action = np.clip(action, [0.0, -1.0], [1.0, 1.0])
        # Stored BEFORE sign flips and the governor: training's "last action" is
        # the policy's own output, not the executed command.
        self._last_action = action.astype(np.float32).copy()

        v = self.v_sign * float(action[0]) * V_MAX
        w = self.w_sign * float(action[1]) * W_MAX

        # --- directional safety governor (no more full freeze) ---------------
        # Block only the velocity direction that drives into a near obstacle;
        # turning stays free so the rover can rotate toward an opening. Thresholds
        # follow the rectangular footprint edge per beam (tight front, wide back),
        # in the rover frame, so it's independent of the drive-sign flips.
        blocked = self._scan < self._stop_range
        if v > 0 and bool(np.any(blocked & self._front_mask)):
            v = 0.0                     # something close ahead -> don't drive forward
        elif v < 0 and bool(np.any(blocked & self._rear_mask)):
            v = 0.0                     # something close behind -> don't reverse
        # Contact imminent (obstacle within the footprint edge anywhere): also
        # stop rotating, as a last-resort guard against clipping a corner.
        if bool(np.any(self._scan < self._hard_range)):
            v = w = 0.0
            self.get_logger().warn("obstacle at footprint -> full stop",
                                   throttle_duration_sec=1.0)

        if self.debug:
            x, y, yaw = self._pose
            dx, dy = self.goal[0] - x, self.goal[1] - y
            bearing = wrap(math.atan2(dy, dx) - yaw)
            self.get_logger().info(
                f"pose=({x:+.2f},{y:+.2f},{math.degrees(yaw):+.0f}deg)  "
                f"dist={dist:.2f}  bearing={math.degrees(bearing):+.0f}deg  "
                f"-> v={v:+.2f} w={w:+.2f}",
                throttle_duration_sec=0.5,
            )

        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.cmd_pub.publish(cmd)

    # ---------------------------------------------------------------- self-test
    def _run_selftest(self):
        """Scripted open-loop check of the drive + turn conventions.

        Phase 0 (0.0-1.0 s): stationary, record start pose.
        Phase 1 (1.0-3.0 s): drive forward at 0.15 m/s   -> expect +x, ~+0.3 m.
        Phase 2 (3.0-4.0 s): stationary.
        Phase 3 (4.0-6.0 s): turn left at +0.5 rad/s      -> expect +yaw (CCW), ~+1 rad.
        Phase 4 (>= 6.0 s):  stop and report the odom deltas, then latch.
        """
        if self._pose is None:
            self._stop("selftest waiting for odom")
            return
        now = self.get_clock().now()
        if self._selftest_t0 is None:
            self._selftest_t0 = now
            self._selftest_p0 = self._pose
            self.get_logger().info("SELFTEST start: forward drive then left turn")
        t = (now - self._selftest_t0).nanoseconds * 1e-9

        cmd = Twist()
        if t < 1.0:
            pass
        elif t < 3.0:
            cmd.linear.x = self.v_sign * 0.15
        elif t < 4.0:
            pass
        elif t < 6.0:
            cmd.angular.z = self.w_sign * 0.5
        else:
            self._stop()
            if not self._reached:   # reuse the latch so we report only once
                x0, y0, yaw0 = self._selftest_p0
                x, y, yaw = self._pose
                dyaw = math.degrees(wrap(yaw - yaw0))
                self.get_logger().info(
                    f"SELFTEST done: forward moved dx={x - x0:+.2f} dy={y - y0:+.2f} m "
                    f"(want +x); turn changed yaw by {dyaw:+.0f} deg (want +, CCW). "
                    f"If a sign is wrong, set -p invert_v / -p invert_w."
                )
                self._reached = True
            return
        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = LeoDeploy()
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
# 0. SANITY CHECKS for the first bring-up (these fix the three classic issues):
#    a) "Is forward actually forward?" Run the open-loop self-test:
#         python3 leo_deploy.py --ros-args -p selftest:=true
#       It drives forward then turns left and prints the odom delta. Want dx>0
#       (forward) and yaw increasing (CCW). If reversed, add -p invert_v:=true
#       and/or -p invert_w:=true -- no retraining needed.
#    b) "Rover goes the wrong way to the goal." Almost always the Jetson (see c)
#       feeding a permanent phantom obstacle, or a lidar-alignment / handedness
#       error. Run with -p debug:=true to watch pose/bearing/v/w and the min beam
#       angle live, and verify fwd/left/right beams respond to a real obstacle.
#    c) "Permanently about to collide." The Jetson on top blocks part of the scan
#       at ~0.15 m. Returns closer than self_clearance (default 0.20 m) are now
#       treated as the rover's own body and ignored. Tune with -p self_clearance.
#
# 1. LIDAR ALIGNMENT is the #1 cause of "it turns the wrong way." Verify that
#    beam index 1600 (relative angle 0) points straight FORWARD on the rover and
#    that your lidar scans CCW. If 0 deg isn't forward, set -p lidar_yaw_offset.
#    Sanity check: put an obstacle dead ahead and confirm self._scan[1600] is small.
#
# 2. train.py now enables domain randomization: lidar noise + beam dropout, one
#    step of actuation latency, and per-episode actuator-gain + lidar-mount
#    jitter. It still uses perfect circular obstacles and a unicycle model, so
#    the remaining gaps are obstacle shape and wheel slip. If the policy won't
#    converge, dial the randomization down (see the make_env() knobs).
#
# 3. /odom drifts. For a fixed goal a few meters away this is fine; for longer
#    runs switch to a map->base_link TF (SLAM/AMCL) and read the goal in /map.
#
# 4. DIAG and LIDAR_MAX_RANGE here MUST equal the training values, not your real
#    room. Changing them silently shifts the policy's inputs out of distribution.
#
# 5. First runs: tether the rover, keep V_MAX low, and have a kill switch on
#    /cmd_vel ready.
