import math

import pygame


class Ball:
    def __init__(self, x=450.0, y=300.0, radius=10.0, friction=0.98):
        self.x = x
        self.y = y
        self.vx = 0.0
        self.vy = 0.0
        self.theta = 0.0
        self.friction = friction
        self.radius = radius
        self.kicked = False

    def kick(self, target_x, target_y, power):
        self.theta = math.atan2(target_y - self.y, target_x - self.x)
        self.vx = math.cos(self.theta) * power
        self.vy = math.sin(self.theta) * power
        self.kicked = True

    def update(self, dt):
        self.x += self.vx * dt
        self.y += self.vy * dt
        self.vx *= self.friction
        self.vy *= self.friction
        if abs(self.vx) < 0.1:
            self.vx = 0.0
        if abs(self.vy) < 0.1:
            self.vy = 0.0

    def draw(self, screen, scale=1.0, offset_x=0, offset_y=0):
        bx = int(self.x * scale) + offset_x
        by = int(self.y * scale) + offset_y
        pygame.draw.circle(screen, (255, 49, 8), (bx, by), max(1, int(self.radius * scale)))


def required_kick_power(distance, dt, friction, safety_factor=1.4):
    """Minimum kick power so the ball (under exponential friction decay)
    travels at least `distance` before coasting to a stop, with margin."""
    return distance * (1 - friction) / dt * safety_factor

