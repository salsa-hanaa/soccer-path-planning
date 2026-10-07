"""Reference planner: ignores every enemy and heads straight for the
target. It exists to prove the harness works end-to-end and as a baseline
that will collide often -- showing why obstacle-aware planning is needed.

Use this file as a template for a real planner: copy it, rename the class,
and change only what `plan()` returns.
"""

from environment.planner_base import BasePlanner
from environment.state import WorldState


class NaiveDirectPlanner(BasePlanner):
    name = "naive"

    def plan(self, state: WorldState) -> tuple[float, float]:
        return state.target_x, state.target_y
