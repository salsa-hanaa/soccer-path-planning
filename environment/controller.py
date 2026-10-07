"""Shared low-level steering controller: the only thing between a planner's
chosen waypoint and the Locomotion layer. Identical for every algorithm, so
none of them can win a comparison by having a better executor -- only by
choosing better waypoints.
"""

import math


def waypoint_to_command(robot, target_x, target_y, max_speed, slow_radius=60.0, k_theta=4.0):
    dx = target_x - robot.x
    dy = target_y - robot.y
    dist = math.hypot(dx, dy)

    if dist < 1e-3:
        return 0.0, 0.0, 0.0

    desired_theta = math.atan2(dy, dx)
    speed = max_speed if dist > slow_radius else max_speed * (dist / slow_radius)

    vx_world = math.cos(desired_theta) * speed
    vy_world = math.sin(desired_theta) * speed

    # heading is purely cosmetic (for drawing) -- just ease it toward travel direction
    angle_diff = (desired_theta - robot.theta + math.pi) % (2 * math.pi) - math.pi
    omega = max(-3.0, min(3.0, angle_diff * k_theta))

    return vx_world, vy_world, omega
