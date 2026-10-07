"""Kinodynamic execution layer shared by every planner.

Translates a desired velocity command into physically feasible motion:
acceleration/angular-acceleration caps plus a step delay that mimics the
reaction lag of a real walking robot. Every planner's output passes through
an identical Locomotion instance, so none of them can "cheat" by teleporting
or turning instantly -- the only thing that differs between algorithms is
the point they choose to steer toward.
"""

import math
from collections import deque


class Locomotion:
    def __init__(
        self,
        max_linear_vel=250.0,
        max_angular_vel=3.0,
        max_accel=150.0,
        max_alpha=5.0,
        step_delay=0.2,
    ):
        self.max_linear_vel = max_linear_vel
        self.max_angular_vel = max_angular_vel
        self.max_accel = max_accel
        self.max_alpha = max_alpha
        self.step_delay = step_delay

        self.vx = 0.0
        self.vy = 0.0
        self.omega = 0.0

        self._buffer = deque()
        self._target_vx = 0.0
        self._target_vy = 0.0
        self._target_omega = 0.0

    def set_command(self, vx, vy, omega, now):
        mag = math.hypot(vx, vy)
        if mag > self.max_linear_vel:
            vx = vx / mag * self.max_linear_vel
            vy = vy / mag * self.max_linear_vel
        omega = max(-self.max_angular_vel, min(self.max_angular_vel, omega))
        self._buffer.append((now, vx, vy, omega))

    def step(self, dt, now):
        while self._buffer and now - self._buffer[0][0] >= self.step_delay:
            _, self._target_vx, self._target_vy, self._target_omega = self._buffer.popleft()

        dvx = self._target_vx - self.vx
        dvy = self._target_vy - self.vy
        domega = self._target_omega - self.omega

        d_linear = math.hypot(dvx, dvy)
        if d_linear > self.max_accel * dt and d_linear > 1e-9:
            self.vx += dvx / d_linear * self.max_accel * dt
            self.vy += dvy / d_linear * self.max_accel * dt
        else:
            self.vx, self.vy = self._target_vx, self._target_vy

        if abs(domega) > self.max_alpha * dt:
            self.omega += math.copysign(self.max_alpha * dt, domega)
        else:
            self.omega = self._target_omega

        return self.vx, self.vy, self.omega
