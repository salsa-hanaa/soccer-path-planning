"""Enemy robots are not planners -- they're scripted, slower moving
obstacles whose only job is to patrol up and down (bouncing between a top
and bottom bound) at a fixed x position, forcing the planner being tested
to route around a blocker.

Each enemy's x anchor is fixed once per episode from the robot's *starting*
position and the target point -- both determined by the seed alone, before
any planner runs. Its vertical bounce only ever depends on its own current
position, never the robot's, so every planner faces an identical obstacle
course on the same seed.
"""

from environment.controller import waypoint_to_command
from environment.locomotion import Locomotion
from environment.robot import RobotBody


class Enemy:
    def __init__(self, anchor_x, y_min, y_max, start_y, start_direction,
                 max_speed=110.0, radius=14.0, sprite=None):
        self.body = RobotBody(anchor_x, start_y, radius=radius, color=(200, 40, 40), sprite=sprite)
        self.locomotion = Locomotion(
            max_linear_vel=max_speed,
            max_angular_vel=2.0,
            max_accel=120.0,
            max_alpha=4.0,
            step_delay=0.05,
        )
        self.anchor_x = anchor_x
        self.y_min = y_min
        self.y_max = y_max
        self.target_y = y_max if start_direction > 0 else y_min
        self.flip_margin = 8.0

    def step(self, dt, now):
        if abs(self.body.y - self.target_y) < self.flip_margin:
            self.target_y = self.y_min if self.target_y == self.y_max else self.y_max

        vx, vy, omega = waypoint_to_command(
            self.body, self.anchor_x, self.target_y, self.locomotion.max_linear_vel
        )
        self.locomotion.set_command(vx, vy, omega, now)
        rvx, rvy, romega = self.locomotion.step(dt, now)
        self.body.integrate(rvx, rvy, romega, dt)
