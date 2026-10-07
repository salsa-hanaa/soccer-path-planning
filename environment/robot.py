import math

import pygame


class RobotBody:
    """Kinematic + rendering state shared by our robot and the enemy robots.

    Velocity commands (vx, vy) are in WORLD frame -- translation is fully
    holonomic and decoupled from heading. (Rotating commands into a body
    frame doesn't mix well with the step_delay in Locomotion: a command
    computed against an old heading would get re-interpreted under a newer
    heading once dequeued, silently steering off-target. Heading/omega here
    is purely cosmetic -- nothing downstream depends on robot.theta.)
    """

    def __init__(self, x, y, theta=0.0, radius=14.0, color=(0, 0, 255)):
        self.x = x
        self.y = y
        self.theta = theta
        self.vx = 0.0
        self.vy = 0.0
        self.omega = 0.0
        self.radius = radius
        self.color = color

    def integrate(self, vx, vy, omega, dt):
        self.vx, self.vy, self.omega = vx, vy, omega
        self.x += vx * dt
        self.y += vy * dt
        self.theta += omega * dt

    def distance_to(self, other_x, other_y):
        return math.hypot(self.x - other_x, self.y - other_y)

    def draw(self, screen, scale=1.0, offset_x=0, offset_y=0):
        px = int(self.x * scale) + offset_x
        py = int(self.y * scale) + offset_y
        r = max(1, int(self.radius * scale))
        pygame.draw.circle(screen, self.color, (px, py), r)
        hx = px + r * math.cos(self.theta)
        hy = py + r * math.sin(self.theta)
        pygame.draw.line(screen, (20, 20, 20), (px, py), (hx, hy), 2)
