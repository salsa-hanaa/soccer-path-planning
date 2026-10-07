"""Data contract exchanged between the environment and a planner.

A planner never touches pygame, physics, or Locomotion directly. Every tick
it receives a WorldState snapshot and must return one target point. See
environment/planner_base.py for the full contract.
"""

from dataclasses import dataclass, field


@dataclass
class EnemyState:
    x: float
    y: float
    vx: float
    vy: float
    radius: float


@dataclass
class WorldState:
    time: float                # waktu simulasi berjalan sejak awal episode (detik)
    dt: float                  # delta time sejak panggilan plan() sebelumnya

    robot_x: float
    robot_y: float
    robot_theta: float
    robot_vx: float
    robot_vy: float
    robot_radius: float

    ball_x: float
    ball_y: float

    target_x: float            # titik positioning yang harus dicapai robot
    target_y: float

    enemies: list[EnemyState] = field(default_factory=list)

    field_width: float = 600.0
    field_length: float = 900.0
