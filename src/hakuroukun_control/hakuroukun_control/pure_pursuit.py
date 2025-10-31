#!/usr/bin/env python3
##
# @file pure_pursuit.py
#
# @brief Provide implementation of pure pursuit controller for
# autonomous driving.
#
# @section author_doxygen_example Author(s)
# - Created by Tran Viet Thanh on 08/12/2024.
#
# Copyright (c) 2024 System Engineering Laboratory.  All rights reserved.

#!/usr/bin/env python3
import math
import numpy as np

class PurePursuit:
    """
    Lightweight Pure Pursuit controller for (N,2) numpy trajectory arrays.
    State = [x, y, yaw]. Returns [v, delta], [tx, ty].
    """

    def __init__(self, trajectory_xy: np.ndarray,
                 wheel_base: float = 1.0,
                 min_speed: float = 0.15,
                 max_speed: float = 0.60,
                 lookahead_min: float = 0.60,
                 lookahead_max: float = 1.20,
                 kappa_max: float = 2.0,
                 max_steer: float = 0.78,     # ~45 deg
                 max_accel: float = 1.0):     # m/s^2 (for limiter)
        assert trajectory_xy.ndim == 2 and trajectory_xy.shape[1] == 2, \
            "trajectory_xy must be (N,2) array of xy points"
        self.traj = trajectory_xy
        self.wheel_base = wheel_base

        self.MIN_SPEED = min_speed
        self.MAX_SPEED = max_speed
        self.lookahead_min = lookahead_min
        self.lookahead_max = lookahead_max
        self.kappa_max = kappa_max
        self.MAX_STEER = max_steer
        self.MAX_ACCEL = max_accel

        self.prev_v = 0.0
        self.last_idx = 0    # nearest point index cache

    # ---- public -------------------------------------------------------------
    def execute(self, state, dt: float = 0.1):
        """
        state: [x, y, yaw]
        dt   : controller period (s) for accel limiting
        """
        idx, Ld = self._search_target_index(state)
        tx, ty = self.traj[idx]

        # heading to target in vehicle frame
        alpha = math.atan2(ty - state[1], tx - state[0]) - state[2]
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))

        # curvature & speed profile
        kappa = abs(2.0 * math.sin(alpha) / max(Ld, 1e-3))
        curv_factor = max(0.0, 1.0 - kappa / self.kappa_max)
        v_des = self.MIN_SPEED + (self.MAX_SPEED - self.MIN_SPEED) * curv_factor

        # acceleration limiter
        dv_max = self.MAX_ACCEL * max(dt, 1e-3)
        v = self.prev_v + np.clip(v_des - self.prev_v, -dv_max, dv_max)
        self.prev_v = v

        # pure pursuit steering with steer clamp
        delta = math.atan2(2.0 * self.wheel_base * math.sin(alpha), Ld)
        delta = float(np.clip(delta, -self.MAX_STEER, self.MAX_STEER))

        return [v, delta], [tx, ty]

    # ---- private ------------------------------------------------------------
    def _search_target_index(self, state):
        """
        Returns (index, lookahead_distance). Uses adaptive lookahead based on current speed.
        """
        # nearest point (advance from last_idx)
        idx = self.last_idx
        # ensure idx in bounds
        idx = max(0, min(idx, len(self.traj) - 1))
        # move forward while next is closer
        cur_d = self._dist_point_state(self.traj[idx], state)
        while (idx + 1) < len(self.traj):
            nxt_d = self._dist_point_state(self.traj[idx + 1], state)
            if cur_d < nxt_d:
                break
            idx += 1
            cur_d = nxt_d
        self.last_idx = idx

        # adaptive lookahead (linear with current speed estimate)
        v_norm = (self.prev_v - self.MIN_SPEED) / (self.MAX_SPEED - self.MIN_SPEED + 1e-6)
        v_norm = max(0.0, min(1.0, v_norm))
        Ld = self.lookahead_min + (self.lookahead_max - self.lookahead_min) * v_norm

        # step forward until distance >= Ld
        d = self._dist_point_state(self.traj[idx], state)
        while d < Ld and (idx + 1) < len(self.traj):
            idx += 1
            d = self._dist_point_state(self.traj[idx], state)

        return idx, max(Ld, 1e-3)

    # ---- helpers ------------------------------------------------------------
    @staticmethod
    def _dist_point_state(pt, state):
        dx = float(pt[0]) - float(state[0])
        dy = float(pt[1]) - float(state[1])
        return math.hypot(dx, dy)

    @staticmethod
    def is_goal(state, traj_xy: np.ndarray, tol: float = 1.0) -> bool:
        dx = float(traj_xy[-1, 0]) - float(state[0])
        dy = float(traj_xy[-1, 1]) - float(state[1])
        return math.hypot(dx, dy) < tol