"""Episode orchestration: owns the ground truth, builds the WorldState
snapshot handed to the planner each tick, executes the chosen waypoint
through the shared controller + Locomotion, scripts the enemies, checks
collision/success/timeout, and records metrics. None of this is the
planner's concern.
"""

import math
import random
import time

import pygame

from environment.assets import load_sprite
from environment.ball import Ball, required_kick_power
from environment.controller import waypoint_to_command
from environment.enemy import Enemy
from environment.field import Field
from environment.locomotion import Locomotion
from environment.metrics import EpisodeResult
from environment.planner_base import BasePlanner
from environment.robot import RobotBody
from environment.state import EnemyState, WorldState

DT = 1.0 / 60.0
ROBOT_RADIUS = 14.0
ENEMY_RADIUS = 14.0
COLLISION_MARGIN = 2.0
SUCCESS_RADIUS = 10.0
TARGET_STANDOFF = 40.0          # R: how far behind the ball the robot must stand
ROBOT_MAX_SPEED = 220.0
MAX_EPISODE_TIME = 45.0         # longer than before: a search phase needs room
FOV_RANGE = 220.0
FOV_ANGLE_DEG = 100.0


def compute_target_point(ball_x, ball_y, goal_x, goal_y, standoff=TARGET_STANDOFF):
    dx, dy = ball_x - goal_x, ball_y - goal_y
    length = math.hypot(dx, dy) or 1e-6
    ux, uy = dx / length, dy / length
    return ball_x + ux * standoff, ball_y + uy * standoff


class SimulationEnv:
    def __init__(self, planner: BasePlanner, render=False):
        self.planner = planner
        self.render = render
        self.field = Field()
        self.ball = Ball()
        self.robot = RobotBody(0, 0, radius=ROBOT_RADIUS, color=(30, 80, 230))
        self.locomotion = Locomotion(max_linear_vel=ROBOT_MAX_SPEED)
        self.enemies: list[Enemy] = []

        self.screen = None
        self.enemy_sprite = None
        if self.render:
            pygame.init()
            self.screen = pygame.display.set_mode(
                (int(self.field.length), int(self.field.width))
            )
            pygame.display.set_caption("Path Planning Sim")
            self.clock = pygame.time.Clock()
            self.font = pygame.font.SysFont("monospace", 16)
            self.banner_font = pygame.font.SysFont("monospace", 72, bold=True)

            self.robot.sprite = load_sprite("team.png", int(ROBOT_RADIUS * 2))
            self.enemy_sprite = load_sprite("enemy.png", int(ENEMY_RADIUS * 2))
            self.ball.sprite = load_sprite("fifa-ball.png", int(self.ball.radius * 2))

    def reset(self, seed: int):
        rng = random.Random(seed)

        self.ball.x = rng.uniform(self.field.length * 0.35, self.field.length * 0.75)
        self.ball.y = rng.uniform(self.field.width * 0.25, self.field.width * 0.75)
        self.ball.vx = self.ball.vy = 0.0
        self.ball.kicked = False

        self.target_x, self.target_y = compute_target_point(
            self.ball.x, self.ball.y, *self.field.goal_center
        )

        self.robot.x = rng.uniform(40, self.field.length * 0.25)
        self.robot.y = rng.uniform(40, self.field.width - 40)
        self.robot.theta = rng.uniform(0, 2 * math.pi)
        self.locomotion = Locomotion(max_linear_vel=ROBOT_MAX_SPEED)
        self.ball_known = False

        corridor_dx = self.target_x - self.robot.x
        corridor_dy = self.target_y - self.robot.y
        patrol_margin = ENEMY_RADIUS + 16.0
        field_y_min, field_y_max = patrol_margin, self.field.width - patrol_margin

        self.enemies = []
        for frac in (0.3, 0.5, 0.7):
            # Patrol band is centered on where the robot's direct line to its
            # target actually crosses this x -- not the whole field height --
            # so the enemy regularly passes through the straight-line path
            # instead of only rarely being nearby.
            anchor_x = self.robot.x + corridor_dx * frac
            crossing_y = self.robot.y + corridor_dy * frac
            band = rng.uniform(60.0, 90.0)
            y_min = max(field_y_min, crossing_y - band)
            y_max = min(field_y_max, crossing_y + band)
            if y_min >= y_max:
                y_min, y_max = field_y_min, field_y_max

            start_y = rng.uniform(y_min, y_max)
            start_direction = rng.choice((-1, 1))
            self.enemies.append(
                Enemy(anchor_x, y_min, y_max, start_y, start_direction, sprite=self.enemy_sprite)
            )

        self.time = 0.0
        self.path_length = 0.0
        self.plan_calls = 0
        self.plan_time_total = 0.0
        self.plan_time_max = 0.0
        self.min_enemy_distance = float("inf")
        self.straight_line_distance = math.hypot(
            self.target_x - self.robot.x, self.target_y - self.robot.y
        )

        self.planner.reset(self._build_state())

    def _in_fov(self, x, y):
        dx, dy = x - self.robot.x, y - self.robot.y
        dist = math.hypot(dx, dy)
        if dist > FOV_RANGE:
            return False
        angle_to = math.atan2(dy, dx)
        angle_diff = abs((angle_to - self.robot.theta + math.pi) % (2 * math.pi) - math.pi)
        return angle_diff <= math.radians(FOV_ANGLE_DEG) / 2

    def _build_state(self) -> WorldState:
        if not self.ball_known and self._in_fov(self.ball.x, self.ball.y):
            self.ball_known = True

        visible_enemies = [
            EnemyState(e.body.x, e.body.y, e.body.vx, e.body.vy, ENEMY_RADIUS)
            for e in self.enemies
            if self._in_fov(e.body.x, e.body.y)
        ]

        return WorldState(
            time=self.time,
            dt=DT,
            robot_x=self.robot.x,
            robot_y=self.robot.y,
            robot_theta=self.robot.theta,
            robot_vx=self.robot.vx,
            robot_vy=self.robot.vy,
            robot_radius=ROBOT_RADIUS,
            ball_x=self.ball.x if self.ball_known else None,
            ball_y=self.ball.y if self.ball_known else None,
            target_x=self.target_x if self.ball_known else None,
            target_y=self.target_y if self.ball_known else None,
            enemies=visible_enemies,
            fov_range=FOV_RANGE,
            fov_angle_deg=FOV_ANGLE_DEG,
            field_width=self.field.width,
            field_length=self.field.length,
        )

    def _step_physics(self, waypoint):
        now = self.time
        prev_x, prev_y = self.robot.x, self.robot.y

        vx, vy, omega = waypoint_to_command(self.robot, waypoint[0], waypoint[1], ROBOT_MAX_SPEED)
        self.locomotion.set_command(vx, vy, omega, now)
        rvx, rvy, romega = self.locomotion.step(DT, now)
        self.robot.integrate(rvx, rvy, romega, DT)

        self.robot.x = max(ROBOT_RADIUS, min(self.field.length - ROBOT_RADIUS, self.robot.x))
        self.robot.y = max(ROBOT_RADIUS, min(self.field.width - ROBOT_RADIUS, self.robot.y))

        self.path_length += math.hypot(self.robot.x - prev_x, self.robot.y - prev_y)

        for enemy in self.enemies:
            enemy.step(DT, now)

        if self.ball.kicked:
            self.ball.update(DT)

        self.time += DT

    def _check_termination(self):
        for enemy in self.enemies:
            d = self.robot.distance_to(enemy.body.x, enemy.body.y)
            clearance = d - (ROBOT_RADIUS + ENEMY_RADIUS)
            self.min_enemy_distance = min(self.min_enemy_distance, clearance)
            if clearance < -COLLISION_MARGIN:
                return "collision"

        if self.robot.distance_to(self.target_x, self.target_y) < SUCCESS_RADIUS:
            return "success"

        if self.time >= MAX_EPISODE_TIME:
            return "timeout"

        return None

    def _fire_kick(self):
        gx, gy = self.field.goal_center
        dist = math.hypot(gx - self.ball.x, gy - self.ball.y)
        power = required_kick_power(dist, DT, self.ball.friction)
        self.ball.kick(gx, gy, power)

    def run_episode(self, seed: int, planner_name: str) -> EpisodeResult:
        self.reset(seed)
        outcome = None

        while outcome is None:
            state = self._build_state()

            t0 = time.perf_counter()
            waypoint = self.planner.plan(state)
            elapsed = time.perf_counter() - t0
            self.plan_calls += 1
            self.plan_time_total += elapsed
            self.plan_time_max = max(self.plan_time_max, elapsed)

            self._step_physics(waypoint)
            outcome = self._check_termination()

            if self.render:
                self._draw()
                if outcome is None:
                    self.clock.tick(60)

        if outcome == "success":
            self._fire_kick()
            if self.render:
                for _ in range(90):
                    self.ball.update(DT)
                    self._draw()
                    self.clock.tick(60)
                    if self.field.ball_crossed_goal_line(self.ball.x, self.ball.y):
                        break
                self._show_banner("GOAL!", (80, 220, 90))
        elif self.render:
            self._show_banner("FAILED", (220, 60, 60))

        return EpisodeResult(
            planner_name=planner_name,
            seed=seed,
            success=(outcome == "success"),
            fail_reason=None if outcome == "success" else outcome,
            elapsed_time=self.time,
            path_length=self.path_length,
            straight_line_distance=self.straight_line_distance,
            plan_calls=self.plan_calls,
            plan_time_total=self.plan_time_total,
            plan_time_max=self.plan_time_max,
            min_enemy_distance=self.min_enemy_distance,
        )

    def _draw(self):
        self.screen.fill((0, 0, 0))
        self.field.draw(self.screen)
        self._draw_fov()

        if self.ball_known:
            pygame.draw.circle(
                self.screen, (255, 230, 0), (int(self.target_x), int(self.target_y)), 6, 2
            )
        self.ball.draw(self.screen)
        for enemy in self.enemies:
            enemy.body.draw(self.screen)
        self.robot.draw(self.screen)

        status = "ball FOUND" if self.ball_known else "searching..."
        label = self.font.render(
            f"t={self.time:4.1f}s  plans={self.plan_calls}  {status}", True, (255, 255, 255)
        )
        self.screen.blit(label, (10, 10))

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                raise SystemExit

        pygame.display.flip()

    def _show_banner(self, text, color, seconds=1.2):
        overlay = pygame.Surface(self.screen.get_size(), pygame.SRCALPHA)
        overlay.fill((0, 0, 0, 150))

        label = self.banner_font.render(text, True, color)
        rect = label.get_rect(center=(self.screen.get_width() // 2, self.screen.get_height() // 2))

        outline = self.banner_font.render(text, True, (0, 0, 0))
        for dx, dy in ((-2, 0), (2, 0), (0, -2), (0, 2)):
            overlay.blit(outline, outline.get_rect(center=(rect.centerx + dx, rect.centery + dy)))
        overlay.blit(label, rect)

        self.screen.blit(overlay, (0, 0))
        pygame.display.flip()

        for _ in range(int(seconds * 60)):
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    raise SystemExit
            self.clock.tick(60)

    def _draw_fov(self):
        half = math.radians(FOV_ANGLE_DEG) / 2
        left = self.robot.theta - half
        right = self.robot.theta + half
        p0 = (self.robot.x, self.robot.y)
        p1 = (self.robot.x + math.cos(left) * FOV_RANGE, self.robot.y + math.sin(left) * FOV_RANGE)
        p2 = (self.robot.x + math.cos(right) * FOV_RANGE, self.robot.y + math.sin(right) * FOV_RANGE)

        cone = pygame.Surface(self.screen.get_size(), pygame.SRCALPHA)
        pygame.draw.polygon(cone, (255, 255, 120, 40), [p0, p1, p2])
        pygame.draw.line(cone, (255, 255, 120, 120), p0, p1, 1)
        pygame.draw.line(cone, (255, 255, 120, 120), p0, p2, 1)
        self.screen.blit(cone, (0, 0))
