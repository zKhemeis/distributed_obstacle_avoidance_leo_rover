"""
LeoRover2DEnv -- a minimal 2D Gymnasium environment for learning obstacle
avoidance + goal reaching with a differential-drive rover.

Built as a stepping stone toward a real Leo Rover (2D Lidar + ROS 2 /cmd_vel):

  Observation : [ normalized Lidar ranges (n_beams) ]  in [0, 1]
                [ normalized distance to goal       ]  in [0, 1]
                [ sin(bearing), cos(bearing)        ]  in [-1, 1]
                -> all robot-relative, so it transfers to the real robot.

  Action      : normalized [v, omega] in [-1, 1].
                v     -> Twist.linear.x   (forward / backward)
                omega -> Twist.angular.z  (turn on the spot)
                The other 4 Twist fields (linear.y/z, angular.x/y) are always 0
                on a differential-drive robot, so they are NOT part of the
                action space -- see the note at the bottom of this file.

  Kinematics  : unicycle / differential drive.
"""

import math
import numpy as np
import gymnasium as gym
from gymnasium import spaces


class LeoRover2DEnv(gym.Env):
    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 10}

    def __init__(
        self,
        arena_size=(6.0, 6.0),       # (W, H) in meters
        n_beams=500,                  # number of Lidar beams
        lidar_fov=2 * math.pi,       # 360 deg scan (set < 2*pi for a forward arc)
        lidar_max_range=10.0,         # meters
        v_max=0.4,                   # m/s   <-- match the real Leo Rover limit
        w_max=1.0,                   # rad/s <-- match the real Leo Rover limit
        dt=0.1,                      # control period (10 Hz, like the tutorial)
        robot_radius=0.18,           # meters (Leo footprint is ~0.35 m wide)
        goal_tol=0.25,               # "reached the goal" radius
        n_obstacles=5,
        obstacle_radius=(0.2, 0.5),  # (min, max) obstacle radius
        max_steps=400,
        lidar_noise=0.05,             # multiplicative noise std (raise for sim2real)
        render_mode=None,
    ):
        super().__init__()
        self.W, self.H = arena_size
        self.diag = math.hypot(self.W, self.H)
        self.n_beams = n_beams
        self.lidar_fov = lidar_fov
        self.lidar_max_range = lidar_max_range
        self.v_max = v_max
        self.w_max = w_max
        self.dt = dt
        self.robot_radius = robot_radius
        self.goal_tol = goal_tol
        self.n_obstacles = n_obstacles
        self.obstacle_radius = obstacle_radius
        self.max_steps = max_steps
        self.lidar_noise = lidar_noise
        self.render_mode = render_mode

        # Fixed beam angles relative to the robot heading.
        if abs(self.lidar_fov - 2 * math.pi) < 1e-6:
            self._beam_rel = np.linspace(-math.pi, math.pi, n_beams, endpoint=False)
        else:
            self._beam_rel = np.linspace(-self.lidar_fov / 2, self.lidar_fov / 2, n_beams)

        # Observation: lidar (n) + dist (1) + sin/cos bearing (2)
        obs_low = np.concatenate([np.zeros(n_beams), [0.0], [-1.0, -1.0]]).astype(np.float32)
        obs_high = np.concatenate([np.ones(n_beams), [1.0], [1.0, 1.0]]).astype(np.float32)
        self.observation_space = spaces.Box(obs_low, obs_high, dtype=np.float32)

        # Action: normalized [v, omega]
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(2,), dtype=np.float32)

        self.robot = None      # [x, y, theta]
        self.goal = None       # [x, y]
        self.obstacles = []    # list of (cx, cy, r)
        self._steps = 0
        self._last_scan = None
        self._prev_dist = None
        self._fig = self._ax = None

    # ----------------------------------------------------------------- helpers
    def _goal_dist(self):
        return float(math.hypot(self.goal[0] - self.robot[0], self.goal[1] - self.robot[1]))

    def _is_free(self, x, y, clearance):
        if x < clearance or x > self.W - clearance or y < clearance or y > self.H - clearance:
            return False
        for cx, cy, r in self.obstacles:
            if math.hypot(x - cx, y - cy) < r + clearance:
                return False
        return True

    def _sample_free_pose(self):
        rng = self.np_random
        for _ in range(200):
            x = rng.uniform(0, self.W)
            y = rng.uniform(0, self.H)
            if self._is_free(x, y, self.robot_radius + 0.05):
                th = rng.uniform(-math.pi, math.pi)
                return np.array([x, y, th], dtype=np.float32)
        return np.array([self.robot_radius + 0.1, self.robot_radius + 0.1, 0.0], dtype=np.float32)

    def _place_world(self):
        rng = self.np_random
        margin = self.robot_radius + 0.1
        self.obstacles = []
        for _ in range(self.n_obstacles):
            r = rng.uniform(*self.obstacle_radius)
            cx = rng.uniform(r + margin, self.W - r - margin)
            cy = rng.uniform(r + margin, self.H - r - margin)
            self.obstacles.append((cx, cy, r))

        self.robot = self._sample_free_pose()

        for _ in range(200):
            gx = rng.uniform(margin, self.W - margin)
            gy = rng.uniform(margin, self.H - margin)
            far = math.hypot(gx - self.robot[0], gy - self.robot[1]) > 0.4 * self.diag
            if self._is_free(gx, gy, self.goal_tol) and far:
                self.goal = np.array([gx, gy], dtype=np.float32)
                return
        self.goal = np.array([self.W - margin, self.H - margin], dtype=np.float32)

    def _raycast(self):
        px, py, th = self.robot
        angles = th + self._beam_rel
        dx = np.cos(angles)
        dy = np.sin(angles)
        ranges = np.full(self.n_beams, self.lidar_max_range, dtype=np.float64)

        # Walls of the arena box [0, W] x [0, H]. For an interior start point the
        # smallest positive crossing among the 4 lines is the true exit point.
        with np.errstate(divide="ignore", invalid="ignore"):
            for t in ((0 - px) / dx, (self.W - px) / dx, (0 - py) / dy, (self.H - py) / dy):
                valid = t > 1e-9
                ranges = np.where(valid, np.minimum(ranges, t), ranges)

        # Circular obstacles (ray-circle intersection, ray dir is unit -> a = 1).
        for cx, cy, r in self.obstacles:
            fx = px - cx
            fy = py - cy
            b = 2 * (fx * dx + fy * dy)
            c = fx * fx + fy * fy - r * r
            disc = b * b - 4 * c
            hit = disc >= 0
            sqrt_disc = np.sqrt(np.where(hit, disc, 0.0))
            t = (-b - sqrt_disc) / 2.0
            valid = hit & (t > 1e-9)
            ranges = np.where(valid, np.minimum(ranges, t), ranges)

        ranges = np.clip(ranges, 0.0, self.lidar_max_range)
        if self.lidar_noise > 0:
            ranges = ranges * (1 + self.np_random.normal(0, self.lidar_noise, self.n_beams))
            ranges = np.clip(ranges, 0.0, self.lidar_max_range)
        return ranges.astype(np.float32)

    def _get_obs(self):
        norm_ranges = self._last_scan / self.lidar_max_range
        px, py, th = self.robot
        dx = self.goal[0] - px
        dy = self.goal[1] - py
        dist = math.hypot(dx, dy)
        dist_norm = min(dist / self.diag, 1.0)
        bearing = math.atan2(dy, dx) - th
        bearing = math.atan2(math.sin(bearing), math.cos(bearing))  # wrap to [-pi, pi]
        return np.concatenate(
            [norm_ranges, [dist_norm], [math.sin(bearing), math.cos(bearing)]]
        ).astype(np.float32)

    # -------------------------------------------------------------- gym API
    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        self._place_world()
        self._steps = 0
        self._last_scan = self._raycast()
        self._prev_dist = self._goal_dist()
        return self._get_obs(), {}

    def step(self, action):
        action = np.clip(np.asarray(action, dtype=np.float32), -1.0, 1.0)
        v = float(action[0]) * self.v_max
        w = float(action[1]) * self.w_max

        px, py, th = self.robot
        px += v * math.cos(th) * self.dt
        py += v * math.sin(th) * self.dt
        th = math.atan2(math.sin(th + w * self.dt), math.cos(th + w * self.dt))
        self.robot = np.array([px, py, th], dtype=np.float32)

        self._steps += 1
        self._last_scan = self._raycast()
        dist = self._goal_dist()
        min_scan = float(self._last_scan.min())

        reward = 2.0 * (self._prev_dist - dist)   # progress toward goal (dense)
        reward -= 0.01                            # time penalty -> shorter paths
        if min_scan < 0.4:                        # stay-away-from-walls penalty
            reward -= 0.1 * (0.4 - min_scan) / 0.4
        self._prev_dist = dist

        terminated = truncated = False
        collided = self._collision()
        if collided:
            reward -= 10.0
            terminated = True
        elif dist < self.goal_tol:
            reward += 20.0
            terminated = True
        if self._steps >= self.max_steps:
            truncated = True

        info = {"dist": dist, "min_scan": min_scan, "is_success": dist < self.goal_tol}
        return self._get_obs(), reward, terminated, truncated, info

    def _collision(self):
        px, py, _ = self.robot
        rr = self.robot_radius
        if px < rr or px > self.W - rr or py < rr or py > self.H - rr:
            return True
        for cx, cy, r in self.obstacles:
            if math.hypot(px - cx, py - cy) < r + rr:
                return True
        return False

    # --------------------------------------------------------------- render
    def render(self):
        if self.render_mode is None:
            return None
        import matplotlib
        import matplotlib.pyplot as plt

        if self._fig is None:
            if self.render_mode == "rgb_array":
                matplotlib.use("Agg")
            self._fig, self._ax = plt.subplots(figsize=(5, 5))

        ax = self._ax
        ax.clear()
        ax.set_xlim(0, self.W)
        ax.set_ylim(0, self.H)
        ax.set_aspect("equal")

        for cx, cy, r in self.obstacles:
            ax.add_patch(plt.Circle((cx, cy), r, color="0.4"))
        ax.add_patch(plt.Circle(tuple(self.goal), self.goal_tol, color="green", alpha=0.4))

        px, py, th = self.robot
        ax.add_patch(plt.Circle((px, py), self.robot_radius, color="tab:blue"))
        ax.plot([px, px + 0.3 * math.cos(th)], [py, py + 0.3 * math.sin(th)], "k-")

        angles = th + self._beam_rel
        ex = px + self._last_scan * np.cos(angles)
        ey = py + self._last_scan * np.sin(angles)
        for i in range(self.n_beams):
            ax.plot([px, ex[i]], [py, ey[i]], color="red", lw=0.3, alpha=0.4)

        if self.render_mode == "human":
            plt.pause(0.001)
            return None
        self._fig.canvas.draw()
        return np.asarray(self._fig.canvas.buffer_rgba())[..., :3].copy()

    def close(self):
        if self._fig is not None:
            import matplotlib.pyplot as plt
            plt.close(self._fig)
            self._fig = self._ax = None


# ---------------------------------------------------------------------------
# Why the action space is 2D and not the full 6-DOF cmd_vel Twist
# ---------------------------------------------------------------------------
# A Leo Rover is a differential-drive (skid-steer) robot. On the ground it can
# only do two things: drive forward/backward (Twist.linear.x) and rotate in
# place (Twist.angular.z). It physically cannot strafe sideways (linear.y),
# move vertically (linear.z) or roll/pitch (angular.x/y). Those fields exist in
# the geometry_msgs/Twist message but are ignored by the rover's controller.
#
# So the learnable action is just (v, omega). Giving the RL agent 6 dimensions
# would force it to waste exploration on 4 dimensions that do nothing and that
# never transfer to hardware. When you deploy, convert the 2D action back into a
# Twist and publish to /cmd_vel:
#
#   from geometry_msgs.msg import Twist
#   msg = Twist()
#   msg.linear.x  = float(action[0]) * V_MAX      # forward / backward
#   msg.angular.z = float(action[1]) * W_MAX      # turn on the spot
#   # all other fields stay 0.0
#   cmd_vel_pub.publish(msg)