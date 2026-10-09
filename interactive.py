"""Interactive demo: the same SimulationEnv as main.py, but with a side
panel GUI for live exploration -- switch between "know the ball" (informed
search: Greedy/A*/GA all need a known target for their heuristic) and
"blind the ball" (search problem) modes, pause/restart, and manually place
the ball or robot to set up a specific scenario.

Run: python3 interactive.py --planner naive
"""

import argparse
import math
import sys

import pygame

from environment.sim import DT, SimulationEnv
from environment.ui import Button, InputField
from planners import REGISTRY

PANEL_WIDTH = 260


class InteractiveApp:
    def __init__(self, planner_name, seed):
        self.planner_name = planner_name
        self.seed = seed
        self.paused = False
        self.phase = "running"   # "running" -> "kicking" -> "ended"
        self._banner_shown = False
        self.alert_text = None
        self.alert_expire = 0

        # Sticky manual placement: once the user Applies a ball/robot
        # position, Restart reproduces that scenario instead of snapping
        # back to the seed's random spawn. Stays in effect across repeated
        # restarts until the user Applies a different position.
        self.custom_ball = None
        self.custom_robot = None

        # Permanent trail of the robot's visited positions, drawn under
        # it every frame. Only cleared on Restart.
        self.path_trace = []

        planner = REGISTRY[planner_name]()
        self.env = SimulationEnv(planner, render=True, panel_width=PANEL_WIDTH)
        self.env.reset(seed)

        self.panel_x = int(self.env.field.length) + 15
        self._build_widgets()

    def _build_widgets(self):
        env = self.env
        x = self.panel_x
        w = PANEL_WIDTH - 30
        y = 16

        self.mode_btn = Button(
            x, y, w, 32,
            lambda: f"Mode: {'Know Ball' if env.mode == 'know_ball' else 'Blind Ball'}",
            self.toggle_mode,
        )
        y += 42

        half = (w - 10) // 2
        self.restart_btn = Button(x, y, half, 32, "Restart", self.restart)
        self.pause_btn = Button(
            x + half + 10, y, half, 32,
            lambda: "Resume" if self.paused else "Pause",
            self.toggle_pause,
        )
        y += 60

        self.ball_header_y = y
        y += 36
        self.ball_x_field = InputField(x, y, 105, 28, "Ball X", env.ball.x, 0, env.field.length)
        self.ball_y_field = InputField(x + 120, y, 105, 28, "Ball Y", env.ball.y, 0, env.field.width)
        y += 44
        self.apply_ball_btn = Button(x, y, w, 30, "Apply Ball Pos", self.apply_ball)
        y += 56

        self.robot_header_y = y
        y += 36
        self.robot_x_field = InputField(x, y, 105, 28, "Robot X", env.robot.x, 0, env.field.length)
        self.robot_y_field = InputField(x + 120, y, 105, 28, "Robot Y", env.robot.y, 0, env.field.width)
        y += 44
        self.apply_robot_btn = Button(x, y, w, 30, "Apply Robot Pos", self.apply_robot)
        y += 56

        # Coordinates can only be edited/applied once the robot has
        # actually stopped (paused, or the episode ended in success/
        # collision/timeout) -- otherwise a teleport is immediately
        # overwritten by the planner on the very next tick, which looks
        # like "my input didn't do anything".
        self.placement_fields = [
            self.ball_x_field, self.ball_y_field, self.apply_ball_btn,
            self.robot_x_field, self.robot_y_field, self.apply_robot_btn,
        ]

        self.info_y = y

        self.widgets = [
            self.mode_btn, self.restart_btn, self.pause_btn,
            self.ball_x_field, self.ball_y_field, self.apply_ball_btn,
            self.robot_x_field, self.robot_y_field, self.apply_robot_btn,
        ]

    def is_stopped(self):
        """True once the robot is no longer actively driving: paused, or
        the episode ended (success/collision/timeout)."""
        return self.paused or self.phase != "running"

    def show_alert(self, text, ms=2200):
        self.alert_text = text
        self.alert_expire = pygame.time.get_ticks() + ms

    def toggle_mode(self):
        if not self.is_stopped():
            self.show_alert("Pause dulu -- masih di tengah permainan")
            return
        self.env.set_mode("know_ball" if self.env.mode == "blind_ball" else "blind_ball")
        self.show_alert("Mode diganti -- klik Restart untuk menerapkan")

    def toggle_pause(self):
        self.paused = not self.paused

    def restart(self):
        self.env.reset(self.seed, ball_pos=self.custom_ball, robot_pos=self.custom_robot)
        self.paused = False
        self.phase = "running"
        self._banner_shown = False
        self.path_trace = []
        self.ball_x_field.set_value(self.env.ball.x)
        self.ball_y_field.set_value(self.env.ball.y)
        self.robot_x_field.set_value(self.env.robot.x)
        self.robot_y_field.set_value(self.env.robot.y)

    def apply_ball(self):
        self.env.set_ball_position(self.ball_x_field.get_value(), self.ball_y_field.get_value())
        self.custom_ball = (self.env.ball.x, self.env.ball.y)
        self.ball_x_field.set_value(self.env.ball.x)
        self.ball_y_field.set_value(self.env.ball.y)

    def apply_robot(self):
        self.env.set_robot_position(self.robot_x_field.get_value(), self.robot_y_field.get_value())
        self.custom_robot = (self.env.robot.x, self.env.robot.y)
        self.robot_x_field.set_value(self.env.robot.x)
        self.robot_y_field.set_value(self.env.robot.y)

    def draw_panel(self):
        screen, font = self.env.screen, self.env.font

        for widget in self.widgets:
            widget.draw(screen, font)

        screen.blit(font.render("Ball position", True, (200, 200, 200)), (self.panel_x, self.ball_header_y))
        screen.blit(font.render("Robot position", True, (200, 200, 200)), (self.panel_x, self.robot_header_y))
        if not self.is_stopped():
            hint = font.render("(pause to edit)", True, (255, 180, 60))
            screen.blit(hint, (self.panel_x + 140, self.ball_header_y))

        dist = math.hypot(self.env.robot.x - self.env.ball.x, self.env.robot.y - self.env.ball.y)
        lines = [
            f"State: {self.env.status_text()}",
            f"Robot: ({self.env.robot.x:5.1f}, {self.env.robot.y:5.1f})",
            f"Dist to ball: {dist:6.1f}",
        ]
        if self.paused:
            lines.append("-- PAUSED --")

        for i, line in enumerate(lines):
            color = (255, 220, 60) if line == "-- PAUSED --" else (255, 255, 255)
            screen.blit(font.render(line, True, color), (self.panel_x, self.info_y + i * 22))

    def draw_alert(self):
        if self.alert_text is None or pygame.time.get_ticks() >= self.alert_expire:
            return
        screen, font = self.env.screen, self.env.font

        text_surf = font.render(self.alert_text, True, (255, 255, 255))
        pad_x, pad_y = 14, 8
        box_w = text_surf.get_width() + pad_x * 2
        box_h = text_surf.get_height() + pad_y * 2
        box_x = (int(self.env.field.length) - box_w) // 2
        box_y = 40

        box = pygame.Surface((box_w, box_h), pygame.SRCALPHA)
        box.fill((40, 40, 40, 235))
        pygame.draw.rect(box, (255, 180, 60), box.get_rect(), 2, border_radius=6)
        box.blit(text_surf, (pad_x, pad_y))
        screen.blit(box, (box_x, box_y))

    def _advance(self):
        if self.paused:
            return

        if self.phase == "running":
            self.env.tick()
            self.path_trace.append((self.env.robot.x, self.env.robot.y))
            if self.env.outcome == "success":
                self.env._fire_kick()
                self.phase = "kicking"
            elif self.env.outcome is not None:
                self.phase = "ended"
        elif self.phase == "kicking":
            self.env.ball.update(DT)
            if self.env.field.ball_crossed_goal_line(self.env.ball.x, self.env.ball.y):
                self.phase = "ended"

        if self.phase == "ended" and not self._banner_shown:
            self._banner_shown = True
            self.env.draw(trace=self.path_trace)
            self.draw_panel()
            pygame.display.flip()
            if self.env.outcome == "success":
                self.env.show_banner("GOAL!", (80, 220, 90))
            else:
                self.env.show_banner("FAILED", (220, 60, 60))

    def run(self):
        while True:
            stopped = self.is_stopped()
            for field in self.placement_fields:
                field.enabled = stopped

            events = self.env.pump_events()
            for event in events:
                for widget in self.widgets:
                    widget.handle_event(event)

            self._advance()

            self.env.draw(flip=False, trace=self.path_trace)
            self.draw_panel()
            self.draw_alert()
            pygame.display.flip()
            self.env.clock.tick(60)


def main():
    parser = argparse.ArgumentParser(description="Interactive demo with a side-panel GUI.")
    parser.add_argument("--planner", default="naive", choices=sorted(REGISTRY.keys()))
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args()

    InteractiveApp(args.planner, args.seed).run()


if __name__ == "__main__":
    sys.exit(main())
