"""
Plain Greedy orders its search queue by h(n) = distance to the target.
This version modifies the heuristic so that cells close to an enemy look worse:

        priority(n) = h(n) + RISK_WEIGHT * risk(n)

PIPELINE (every tick)
    1. REMEMBER enemies : keep the last seen position of every enemy for a few
                          seconds (the FOV is narrow, enemies leave it often).
    2. BUILD RISK ZONES : each remembered enemy gets a vertical segment that
                          grows with the time since it was seen.
    3. SEARCH           : Greedy Best-First on a 30 px grid, every REPLAN_EVERY ticks.
    4. FOLLOW           : steer toward the next waypoint of the cached path.
    5. ARRIVED          : at the target, aim at the ball (= face the goal).

Designed for know_ball mode (the target position is always known).
"""

import heapq
import math

from environment.planner_base import BasePlanner
from environment.state import WorldState


class GreedyPlanner(BasePlanner):
    name = "greedy"

    # grid and path following 
    CELL = 30.0                 # grid cell size (px)
    REPLAN_EVERY = 10           # re-run the search every N ticks, reuse the path between
    WAYPOINT_TOL = CELL * 0.5   # a waypoint counts as reached within this distance (px)

    # risk map
    RISK_WEIGHT = 1.0                           # how much risk matters compared to distance
    CORE_RADIUS, CORE_PENALTY = 40.0, 500.0     # right next to the enemy
    NEAR_RADIUS, NEAR_PENALTY = 70.0, 100.0     # close
    FAR_RADIUS, FAR_PENALTY = 110.0, 10.0       # keep a respectful distance

    # enemy memory
    MEMORY_MAX_AGE = 3.0        # seconds an unseen enemy is still remembered
    HORIZON = 1.0               # seconds of enemy motion assumed even for visible enemies
    ENEMY_MAX_SPEED = 110.0     # px/s (from enemy.py)
    MAX_SPREAD = 180.0          # px, cap on the vertical uncertainty of an enemy

    def reset(self, state: WorldState) -> None:
        """Start of an episode: forget enemies and the cached path."""
        self._memory = {}
        self._path = []
        self._tick = 0

    def plan(self, state: WorldState) -> tuple[float, float]:
        """Called every tick. Returns the (x, y) point the robot should head to."""
        self._cols = max(1, int(state.field_length / self.CELL))
        self._rows = max(1, int(state.field_width / self.CELL))
        self._update_memory(state)                                  

        # Blind ball: head to the field center.
        if state.target_x is None:
            return (state.field_length / 2, state.field_width / 2)

        zones = self._risk_zones(state)                             

        if self._tick % self.REPLAN_EVERY == 0 or not self._path:    
            self._path = self._greedy_search(state, zones)
        self._tick += 1

        # Drop waypoints that were already reached.
        while self._path and math.hypot(state.robot_x - self._path[0][0],
                                        state.robot_y - self._path[0][1]) < self.WAYPOINT_TOL:
            self._path.pop(0)

        # The path always ends exactly at the target, so when it is used up we are there -> aim at the ball to face the goal.
        if not self._path:
            return (state.target_x, state.target_y)
        return self._path[0]

    # Enemy memory and risk zones
    def _update_memory(self, state: WorldState) -> None:
        """Store the last seen position of each visible enemy; drop old entries."""
        for e in state.enemies:
            self._memory[round(e.x, 1)] = (e.x, e.y, e.radius, state.time)   # key: fixed enemy x
        for key in [k for k, m in self._memory.items()
                    if state.time - m[3] > self.MEMORY_MAX_AGE]:
            del self._memory[key]

    def _risk_zones(self, state: WorldState):
        """
        One zone per remembered enemy: (x, y_min, y_max, touch_distance).
        Enemies patrol up and down, so the longer unseen, the longer the segment:
            spread = enemy_max_speed * (age + HORIZON)
        """
        zones = []
        for x, y, radius, seen_at in self._memory.values():
            age = state.time - seen_at
            spread = min(self.ENEMY_MAX_SPEED * (age + self.HORIZON), self.MAX_SPREAD)
            zones.append((x, max(0.0, y - spread), min(state.field_width, y + spread),
                          radius + state.robot_radius))
        return zones

    def _risk(self, px, py, zones) -> float:
        """Total risk at a point: layered penalty by distance to each enemy segment."""
        total = 0.0
        for ex, y_min, y_max, touch in zones:
            dy = 0.0 if y_min <= py <= y_max else min(abs(py - y_min), abs(py - y_max))
            d = math.hypot(px - ex, dy)
            if d < touch + self.CORE_RADIUS:
                total += self.CORE_PENALTY
            elif d < touch + self.NEAR_RADIUS:
                f = (d - touch - self.CORE_RADIUS) / (self.NEAR_RADIUS - self.CORE_RADIUS)
                total += self.CORE_PENALTY + (self.NEAR_PENALTY - self.CORE_PENALTY) * f
            elif d < touch + self.FAR_RADIUS:
                f = (d - touch - self.NEAR_RADIUS) / (self.FAR_RADIUS - self.NEAR_RADIUS)
                total += self.NEAR_PENALTY + (self.FAR_PENALTY - self.NEAR_PENALTY) * f
        return total

    # Greedy Best-First Search with the risk-aware heuristic
    def _greedy_search(self, state: WorldState, zones):
        """Returns a list of (x, y) waypoints from the robot to the target."""
        start = self._to_cell(state.robot_x, state.robot_y)
        goal = self._to_cell(state.target_x, state.target_y)

        priority_cache = {}

        def priority(c):                       # h(n) + RISK_WEIGHT * risk(n)
            if c not in priority_cache:
                x, y = self._to_xy(c)
                priority_cache[c] = (self._distance_estimate(c, goal)
                                     + self.RISK_WEIGHT * self._risk(x, y, zones))
            return priority_cache[c]

        queue = [(priority(start), 0, start)]  # (priority, tie-breaker, cell)
        came_from = {start: None}
        counter = 1

        while queue:
            _, _, current = heapq.heappop(queue)       # best (lowest) priority first
            if current == goal:
                return self._build_waypoints(current, came_from, state)
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    if dx == 0 and dy == 0:
                        continue
                    nb = (current[0] + dx, current[1] + dy)
                    if not (0 <= nb[0] < self._cols and 0 <= nb[1] < self._rows):
                        continue
                    if nb in came_from:
                        continue
                    came_from[nb] = current
                    heapq.heappush(queue, (priority(nb), counter, nb))
                    counter += 1

        return [(state.target_x, state.target_y)]

    def _distance_estimate(self, a, b) -> float:
        """Octile distance (exact straight-line cost on an 8-direction grid), in px."""
        dx, dy = abs(a[0] - b[0]), abs(a[1] - b[1])
        return self.CELL * (max(dx, dy) + (math.sqrt(2) - 1) * min(dx, dy))

    def _build_waypoints(self, cell, came_from, state: WorldState):
        """Walk back through came_from, then convert cells to world points."""
        cells = []
        while cell is not None:
            cells.append(cell)
            cell = came_from[cell]
        cells.reverse()
        points = [self._to_xy(c) for c in cells[1:]]   # skip the robot's own cell

        target = (state.target_x, state.target_y)
        while points and math.hypot(points[-1][0] - target[0], points[-1][1] - target[1]) < self.CELL:
            points.pop()                               # last cell is replaced by the exact target
        points.append(target)
        return points

    # Grid <-> world conversion
    def _to_cell(self, x, y):
        return (min(self._cols - 1, max(0, int(x // self.CELL))),
                min(self._rows - 1, max(0, int(y // self.CELL))))

    def _to_xy(self, cell):
        return (cell[0] * self.CELL + self.CELL / 2.0,
                cell[1] * self.CELL + self.CELL / 2.0)