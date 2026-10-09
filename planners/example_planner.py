"""Reference planner: searches with a dumb spinning creep until the ball
enters FOV, then beelines for the target ignoring every enemy. It exists to
prove the harness works end-to-end and as a baseline that will search
poorly and collide often -- showing why real search + obstacle-aware
planning is needed.

Use this file as a template for a real planner: copy it, rename the class,
and change what `plan()` does in each branch. `state.target_x` (and
`ball_x`) are None until the ball has entered the robot's FOV at least
once -- a real planner should explore deliberately instead of this
placeholder creep-and-spin.
"""

import math

from environment.planner_base import BasePlanner
from environment.state import WorldState


class NaiveDirectPlanner(BasePlanner):
    name = "naive"

    def plan(self, state: WorldState) -> tuple[float, float]:
        if state.target_x is None:
            # Not a real search strategy -- just enough to keep moving
            # (and sweep the FOV cone a little) so the harness stays valid.
            # Replace this branch with actual exploration logic.
            ahead = state.robot_theta + 0.4
            return (
                state.robot_x + math.cos(ahead) * 60,
                state.robot_y + math.sin(ahead) * 60,
            )

        return state.target_x, state.target_y
