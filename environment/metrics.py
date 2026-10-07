import json
import statistics
from dataclasses import asdict, dataclass


@dataclass
class EpisodeResult:
    planner_name: str
    seed: int
    success: bool
    fail_reason: str | None   # "collision", "timeout", or None
    elapsed_time: float
    path_length: float
    straight_line_distance: float
    plan_calls: int
    plan_time_total: float
    plan_time_max: float
    min_enemy_distance: float


class MetricsLog:
    def __init__(self):
        self.episodes: list[EpisodeResult] = []

    def add(self, result: EpisodeResult):
        self.episodes.append(result)

    def summary(self, planner_name: str | None = None) -> dict:
        rows = [e for e in self.episodes if planner_name is None or e.planner_name == planner_name]
        if not rows:
            return {}

        n = len(rows)
        successes = [e for e in rows if e.success]
        collisions = [e for e in rows if e.fail_reason == "collision"]
        timeouts = [e for e in rows if e.fail_reason == "timeout"]

        efficiency = [
            e.path_length / e.straight_line_distance
            for e in successes
            if e.straight_line_distance > 0
        ]

        return {
            "episodes": n,
            "success_rate": len(successes) / n,
            "collision_rate": len(collisions) / n,
            "timeout_rate": len(timeouts) / n,
            "avg_time_to_target": statistics.fmean(e.elapsed_time for e in successes) if successes else None,
            "avg_path_efficiency": statistics.fmean(efficiency) if efficiency else None,
            "avg_plan_time_ms": statistics.fmean(e.plan_time_total / max(e.plan_calls, 1) for e in rows) * 1000,
            "max_plan_time_ms": max((e.plan_time_max for e in rows), default=0.0) * 1000,
            "avg_min_enemy_distance": statistics.fmean(e.min_enemy_distance for e in rows),
        }

    def save_json(self, path: str):
        with open(path, "w") as f:
            json.dump([asdict(e) for e in self.episodes], f, indent=2)
