"""A* minimalis untuk soccer robot.

f(n) = g(n) + h(n)
  g(n) = jarak tempuh + penalti enemy (kedekatan + arah gerak enemy)
  h(n) = jarak sel ke target (heuristik octile)

Obstacle: setiap sel yang terlalu dekat dengan enemy diberi biaya tambahan,
bukan diblokir mati -- jadi A* tidak pernah buntu.
"""

import heapq
import math
from environment.planner_base import BasePlanner
from environment.state import WorldState

CELL = 30.0                     # ukuran sel (px)
SAFE_RADIUS = 45.0              # radius bahaya di luar radius fisik
CLOSE_PENALTY = 400.0           # penalti maksimum karena kedekatan
DIRECTION_PENALTY = 3.0         # penalti tambahan karena enemy mendekat
REPLAN_EVERY = 10                # replan tiap N tick
WAYPOINT_TOL = CELL * 0.5

# --- layer penalti -------------------------------------------------
# makin dekat, makin mahal, dengan lompatan besar di "inti" bahaya
CORE_RADIUS = 40.0        # inti: sangat mahal, hampir terlarang
CORE_PENALTY = 500.0     # biaya datar di dalam inti
NEAR_RADIUS = 70.0        # zona dekat: cukup mahal
NEAR_PENALTY = 100.0      # biaya di tepi zona dekat
FAR_RADIUS = 110.0        # zona jauh: sedikit mahal, biar A* "menghormati"
FAR_PENALTY = 10.0       # biaya di tepi zona jauh
# -------------------------------------------------------------------

class AStarPlanner(BasePlanner):
    name = "astar"

    def reset(self, state):
        self.path = []
        self.tick = 0
        self.cols = 30
        self.rows = 20

    def plan(self, state: WorldState) -> tuple[float, float]:
        self.cols = max(1, int(state.field_length / CELL))
        self.rows = max(1, int(state.field_width / CELL))

        # blind_ball: bola belum terlihat -> jalan lurus ke depan sebagai fallback
        if state.target_x is None:
            return (state.robot_x + 60 * math.cos(state.robot_theta),
                    state.robot_y + 60 * math.sin(state.robot_theta))

        # replan berkala atau kalau path kosong
        if self.tick % REPLAN_EVERY == 0 or not self.path:
            self.path = self._astar(state)
        self.tick += 1

        # buang waypoint yang sudah dilewati
        while self.path and self._dist((state.robot_x, state.robot_y), self.path[0]) < WAYPOINT_TOL:
            self.path.pop(0)

        return self.path[0] if self.path else (state.target_x, state.target_y)

    # ------------------------------------------------------------------
    # A* inti
    # ------------------------------------------------------------------
    def _astar(self, state):
        start = self._to_cell(state.robot_x, state.robot_y)
        goal = self._to_cell(state.target_x, state.target_y)

        open_heap = [(self._h(start, goal), 0.0, start)]
        came_from = {}
        g_score = {start: 0.0}
        closed = set()

        while open_heap:
            _, g, cur = heapq.heappop(open_heap)
            if cur in closed:
                continue
            closed.add(cur)

            if cur == goal:
                return self._reconstruct(came_from, cur, state)

            for nb, step in self._neighbors(cur):
                if nb in closed:
                    continue
                new_g = g + step + self._enemy_cost(nb, state)
                if new_g < g_score.get(nb, float("inf")):
                    g_score[nb] = new_g
                    came_from[nb] = cur
                    heapq.heappush(open_heap, (new_g + self._h(nb, goal), new_g, nb))

        return [(state.target_x, state.target_y)]

    def _neighbors(self, cell):
        x, y = cell
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                nx, ny = x + dx, y + dy
                if 0 <= nx < self.cols and 0 <= ny < self.rows:
                    yield (nx, ny), math.hypot(dx, dy) * CELL

    def _h(self, a, b):
        """Heuristik octile: jarak sel a ke sel b, dalam pixel."""
        dx, dy = abs(a[0] - b[0]), abs(a[1] - b[1])
        return CELL * (max(dx, dy) + (math.sqrt(2) - 1) * min(dx, dy))

    def _enemy_cost(self, cell, state):
        """Biaya bertingkat: makin dekat enemy, makin mahal, dengan lompatan
        besar di inti zona bahaya. Plus bonus kalau enemy mendekat."""
        cx, cy = self._to_world(cell)
        total = 0.0


        for e in state.enemies:
            dx, dy = cx - e.x, cy - e.y
            dist = math.hypot(dx, dy)
            touch = e.radius + state.robot_radius

            # --- biaya bertingkat berdasarkan jarak ---
            if dist < touch + CORE_RADIUS:
                total += CORE_PENALTY
            elif dist < touch + NEAR_RADIUS:
                frac = (dist - touch - CORE_RADIUS) / (NEAR_RADIUS - CORE_RADIUS)
                total += CORE_PENALTY + (NEAR_PENALTY - CORE_PENALTY) * frac
            elif dist < touch + FAR_RADIUS:
                frac = (dist - touch - NEAR_RADIUS) / (FAR_RADIUS - NEAR_RADIUS)
                total += NEAR_PENALTY + (FAR_PENALTY - NEAR_PENALTY) * frac

        return total

    def _reconstruct(self, came_from, cur, state):
        cells = [cur]
        while cur in came_from:
            cur = came_from[cur]
            cells.append(cur)
        cells.reverse()
        pts = [self._to_world(c) for c in cells[1:]]

        target = (state.target_x, state.target_y)
        # Buang waypoint yang terlalu dekat dengan target asli
        while pts and self._dist(pts[-1], target) < CELL:
            pts.pop()
        # Selalu akhiri dengan target presisi
        if not pts or self._dist(pts[-1], target) > 1.0:
            pts.append(target)
        return pts

    # ------------------------------------------------------------------
    # Konversi grid <-> world
    # ------------------------------------------------------------------
    def _to_cell(self, x, y):
        return (min(self.cols - 1, max(0, int(x // CELL))),
                min(self.rows - 1, max(0, int(y // CELL))))

    def _to_world(self, cell):
        return (cell[0] * CELL + CELL / 2.0,
                cell[1] * CELL + CELL / 2.0)

    @staticmethod
    def _dist(a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])