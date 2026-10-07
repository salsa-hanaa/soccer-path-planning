"""Enemy robots are not planners -- they're scripted, slower moving
obstacles whose only job is to loiter somewhere along the robot's approach
corridor, forcing the planner being tested to route around a blocker.

Each enemy's base position is fixed once per episode from the robot's
*starting* position and the target point -- both of which are determined by
the seed alone, before any planner runs. Its only time-varying motion is a
slow perpendicular wobble driven purely by elapsed time. None of this ever
reads the robot's live position, so every planner faces an identical
obstacle course on the same seed.
"""

import math

from environment.controller import waypoint_to_command
from environment.locomotion import Locomotion
from environment.robot import RobotBody


class Enemy:
    def __init__(self, base_x, base_y, perp_x, perp_y, amplitude, period, phase,
                 max_speed=110.0, radius=14.0):
        self.body = RobotBody(base_x, base_y, radius=radius, color=(200, 40, 40))
        self.locomotion = Locomotion(
            max_linear_vel=max_speed,
            max_angular_vel=2.0,
            max_accel=120.0,
            max_alpha=4.0,
            step_delay=0.05,
        )
        self.base_x = base_x
        self.base_y = base_y
        self.perp_x = perp_x
        self.perp_y = perp_y
        self.amplitude = amplitude
        self.period = period
        self.phase = phase

    def patrol_point(self, now):
        wobble = self.amplitude * math.sin(2 * math.pi * now / self.period + self.phase)
        return self.base_x + self.perp_x * wobble, self.base_y + self.perp_y * wobble

    def step(self, dt, now):
        tx, ty = self.patrol_point(now)
        vx, vy, omega = waypoint_to_command(self.body, tx, ty, self.locomotion.max_linear_vel)
        self.locomotion.set_command(vx, vy, omega, now)
        rvx, rvy, romega = self.locomotion.step(dt, now)
        self.body.integrate(rvx, rvy, romega, dt)
