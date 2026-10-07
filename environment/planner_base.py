"""The contract every teammate's path planning algorithm must implement.

Only one method is required: plan(state) -> (x, y). The environment calls it
on a fixed schedule, times every call automatically, and drives the robot
toward whatever point is returned using its own waypoint-following
controller + Locomotion model. This keeps execution identical across every
algorithm plugged in, so differences in the resulting metrics are
attributable to planning quality alone.

A planner may recompute a fresh answer on every call (cheap, reactive
strategies) or cache an internally computed plan and only recompute every
few calls (expensive, global strategies) -- that choice is part of the
algorithm's own design and is exactly what the computation-time metric is
meant to surface. The environment does not care which strategy is used.
"""

from abc import ABC, abstractmethod

from environment.state import WorldState


class BasePlanner(ABC):
    name: str = "base"

    def reset(self, state: WorldState) -> None:
        """Called once at the start of each episode. Optional to override."""

    @abstractmethod
    def plan(self, state: WorldState) -> tuple[float, float]:
        """Return the (x, y) point the robot should head toward next.

        Must never return a point that requires knowledge the planner
        wasn't given in `state` (e.g. no peeking at other episodes).
        """
        raise NotImplementedError
