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
    robot_theta: float         # also the facing direction the FOV cone points in
    robot_vx: float
    robot_vy: float
    robot_radius: float

    # "know_ball": ball_x/y and target_x/y are always populated -- only the
    #   enemy list is FOV-gated. Use this for the informed algorithms
    #   (Greedy, A*, GA) that need a known target to compute a heuristic.
    # "blind_ball": ball_x/y and target_x/y are None until the ball enters
    #   the FOV cone -- a genuine search problem.
    mode: str

    # None until the ball has entered the robot's FOV at least once this
    # episode (always populated in "know_ball" mode). Once discovered,
    # these stay populated for the rest of the episode (the ball doesn't
    # move during the approach).
    ball_x: float | None
    ball_y: float | None

    # The positioning point behind the ball, lined up with the goal. Also
    # None until the ball has been discovered -- there's nothing to head
    # toward yet.
    target_x: float | None
    target_y: float | None

    # Only the enemies currently inside the FOV cone -- there may be 0 to 3
    # of them. An enemy outside this list can still physically collide with
    # the robot; not seeing it doesn't mean it isn't there.
    enemies: list[EnemyState] = field(default_factory=list)

    fov_range: float = 220.0
    fov_angle_deg: float = 100.0

    field_width: float = 600.0
    field_length: float = 900.0
