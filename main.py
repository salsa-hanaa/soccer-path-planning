import argparse
import sys

from environment.metrics import MetricsLog
from environment.sim import SimulationEnv
from planners import REGISTRY


def main():
    parser = argparse.ArgumentParser(description="Run the soccer path-planning sim.")
    parser.add_argument("--planner", default="naive", choices=sorted(REGISTRY.keys()))
    parser.add_argument("--episodes", type=int, default=1)
    parser.add_argument("--seed-start", type=int, default=0)
    parser.add_argument("--render", action="store_true", help="show the pygame window")
    parser.add_argument("--save-json", default=None, help="path to dump per-episode results")
    args = parser.parse_args()

    planner_cls = REGISTRY[args.planner]
    planner = planner_cls()
    env = SimulationEnv(planner, render=args.render)
    log = MetricsLog()

    for i in range(args.episodes):
        seed = args.seed_start + i
        result = env.run_episode(seed, planner.name)
        log.add(result)
        print(
            f"[{i + 1}/{args.episodes}] seed={seed} "
            f"-> {'SUCCESS' if result.success else 'FAIL(' + result.fail_reason + ')'} "
            f"t={result.elapsed_time:.2f}s plans={result.plan_calls}"
        )

    print("\n--- summary ---")
    for key, value in log.summary(planner.name).items():
        print(f"{key}: {value}")

    if args.save_json:
        log.save_json(args.save_json)


if __name__ == "__main__":
    sys.exit(main())
