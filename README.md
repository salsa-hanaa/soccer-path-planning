# Path Planning — Soccer Sim

2D soccer-sim environment for comparing path planning algorithms (Greedy,
A*, GA). Our robot must reach a positioning point beside the ball without
ever colliding with 3 slower, scripted enemy robots whose only job is to
block the way. An enemy outside the robot's FOV can still physically
collide with it; not seeing it doesn't mean it isn't there.

Two modes, switchable per run (see below):
- **`know_ball`** (default for the batch runner): the ball/target position
  is always known — matches an informed search algorithm that needs a
  target to compute a heuristic (Greedy/A*/GA all need this). Only the
  enemies stay FOV-gated.
- **`blind_ball`**: the ball is *not* known until it enters the robot's FOV
  cone — a genuine search/exploration problem, for whoever on the team
  wants to explore that instead.

This repo is split so that **algorithm authors never touch physics,
rendering, or scoring** — they only implement one function.

## Running it

```bash
pip install -r requirements.txt

# Batch (headless or rendered), for collecting comparison metrics:
python main.py --planner naive --episodes 10 --render
python main.py --planner naive --mode blind_ball --episodes 200 --save-json results.json

# Interactive demo with a side-panel GUI (mode toggle, pause, restart,
# manually place the ball/robot):
python interactive.py --planner naive --seed 0
```

## Project layout

```
environment/      # physics, rendering, scoring — do not need to be touched
  state.py         # WorldState / EnemyState: the only thing a planner sees
  planner_base.py  # BasePlanner: the only thing a planner must implement
  sim.py           # episode loop, mode switching, collision/success/timeout, metrics
  ui.py            # pygame Button/InputField widgets used by interactive.py
  ...
planners/
  example_planner.py   # NaiveDirectPlanner — copy this as your starting point
  __init__.py           # REGISTRY: add your planner here with a short name
main.py            # batch CLI entry point (metrics collection)
interactive.py     # GUI demo entry point (mode toggle, pause, manual placement)
```

## The contract: what you actually need to write

Copy `planners/example_planner.py`, rename the class, and implement one
method:

```python
from environment.planner_base import BasePlanner
from environment.state import WorldState

class MyPlanner(BasePlanner):
    name = "myalgo"

    def reset(self, state: WorldState) -> None:
        # optional: called once at the start of each episode
        ...

    def plan(self, state: WorldState) -> tuple[float, float]:
        # required: return the (x, y) point the robot should head toward next
        ...
```

Then register it in `planners/__init__.py`:

```python
from planners.my_planner import MyPlanner
REGISTRY = {"naive": NaiveDirectPlanner, "myalgo": MyPlanner}
```

Run it with `python main.py --planner myalgo --render`.

### `WorldState` fields you get every call

| field | meaning |
|---|---|
| `time`, `dt` | simulated time elapsed, and the tick length (1/60 s) |
| `robot_x/y/theta/vx/vy`, `robot_radius` | our robot, ground truth. `robot_theta` is also the direction the FOV cone points in |
| `mode` | `"know_ball"` or `"blind_ball"` — read this if your planner wants to behave differently per mode, though usually checking `ball_x is None` is enough |
| `ball_x/y` | In `know_ball` mode: always populated. In `blind_ball` mode: **`None` until the ball has entered the FOV cone at least once this episode**, then stays populated for the rest of the episode |
| `target_x/y` | Same availability as `ball_x/y` — the point the robot must reach (see "scenario rules") |
| `enemies` | list of `EnemyState(x, y, vx, vy, radius)` for **only the enemies currently inside the FOV cone** — 0 to 3 of them, in *either* mode. One outside this list can still collide with you |
| `fov_range`, `fov_angle_deg` | your own sensing radius and cone angle (centered on `robot_theta`) |
| `field_width`, `field_length` | playable area, (0,0) to (field_length, field_width) |

### Rules of the contract

- `plan()` is called **every tick**. You decide internally whether to
  recompute from scratch or reuse a cached plan — that choice is part of
  your algorithm and exactly what gets compared (see metrics below).
- In `blind_ball` mode, until `ball_x` is no longer `None`, your job is
  **search**: decide where to head to sweep the FOV cone over unexplored
  ground. `BasePlanner` instances persist across calls (and `reset()` fires
  once per episode), so it's natural to track your own "where have I
  already looked" state internally — the environment doesn't hand you an
  exploration map. In `know_ball` mode `ball_x` is never `None`, so this
  branch never triggers.
- You must **never cause a collision** with any enemy, seen or unseen —
  that is scored as a hard failure, not a soft penalty.
- You do not control the robot's velocity directly. The environment steers
  toward whatever point you return, through the same acceleration/step-delay
  model for every planner, so comparisons stay apples-to-apples.
- Enemies are scripted obstacles, not opponents — they never chase the
  ball or react to your live position. Each patrols up and down (bouncing
  between a top and bottom bound, slower than you) at a fixed x position
  derived from the straight line between the robot's *starting* position
  and the (ground-truth) target — frozen at episode reset, so it's
  identical for every planner on the same seed regardless of whether/when
  that planner personally discovers the ball. Predicting their motion is
  optional; only useful if your algorithm plans more than one step ahead.

## Interactive demo (`interactive.py`)

A side-panel GUI next to the field (pygame-native widgets, no extra GUI
toolkit) for exploring a planner's behavior live:

- **Mode button** — flips between `Know Ball` / `Blind Ball`. Only works
  while paused (or after the episode has ended) — clicking it mid-run shows
  an alert instead, since swapping the ball-visibility rule out from under
  a planner that's actively running doesn't mean anything. After switching,
  click **Restart** to actually re-run the episode in the new mode.
- **Restart** — re-runs the same seed from scratch (also applies any mode
  change staged while paused).
- **Pause / Resume** — freezes the simulation; widgets keep responding.
- **Ball X/Y** + **Apply Ball Pos** — teleport the ball to any coordinate to
  set up a specific scenario. Only editable/applicable while paused or
  after the episode has ended, for the same reason as the mode button.
- **Robot X/Y** + **Apply Robot Pos** — same, for our robot.
- Live readout: current state (`SEARCHING` / `APPROACHING` / `SUCCESS` /
  `COLLISION` / `TIMEOUT`), the robot's live coordinates, and the
  (ground-truth) distance to the ball.

This uses the exact same `SimulationEnv`/planner contract as `main.py` —
nothing about the planner interface changes.

## Scenario rules

- The robot starts with a random heading and **does not know where the
  ball is**. It must search using its FOV cone (`fov_range`, `fov_angle_deg`,
  centered on `robot_theta`) until the ball enters it.
- Once discovered, `target_x/y` is placed on the line between the goal and
  the ball, on the far side of the ball from the goal — reaching it means
  the robot ends up lined up to shoot. Kicking itself is **not** part of
  what's scored: once a planner reaches the target, the environment
  auto-kicks the ball straight into the goal (guaranteed, see
  `environment/ball.py:required_kick_power`).
- Episode ends in exactly one of three ways: **success** (reached target),
  **collision** (hit an enemy, seen or not — hard fail), or **timeout** (45s).

## Metrics collected per episode (`environment/metrics.py`)

- success / collision / timeout rate
- path efficiency (actual path length ÷ straight-line distance)
- time to target
- planning computation time per call (avg + max)
- minimum clearance ever kept from an enemy

Run `--episodes 30+` with a fixed `--seed-start` across all three planners
to get comparable numbers for the report.
