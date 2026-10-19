"""Jalankan semua seed, tampilkan HANYA yang gagal, satu per satu.
Tekan SPACE untuk lanjut ke kegagalan berikutnya, ESC untuk keluar,
R untuk mengulang episode yang sama.
"""

import sys
import pygame

from environment.sim import SimulationEnv
from planners import REGISTRY
import argparse


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--planner", default="astar", choices=sorted(REGISTRY.keys()))
    parser.add_argument("--mode", default="know_ball", choices=("know_ball", "blind_ball"))
    parser.add_argument("--episodes", type=int, default=300)
    parser.add_argument("--seed-start", type=int, default=0)
    args = parser.parse_args()

    planner = REGISTRY[args.planner]()
    env = SimulationEnv(planner, render=True)
    env.set_mode(args.mode)

    failures = []

    # Fase 1: jalankan semua episode headless, catat yang gagal
    print(f"Menjalankan {args.episodes} episode headless...")
    env.render = False
    for i in range(args.episodes):
        seed = args.seed_start + i
        result = env.run_episode(seed, planner.name)
        status = "SUCCESS" if result.success else f"FAIL({result.fail_reason})"
        print(f"[{i+1}/{args.episodes}] seed={seed} -> {status} t={result.elapsed_time:.2f}s")
        if not result.success:
            failures.append({
                "seed": seed,
                "reason": result.fail_reason,
                "time": result.elapsed_time,
                "plans": result.plan_calls,
            })

    print(f"\nTotal gagal: {len(failures)}")
    for f in failures:
        print(f"  seed={f['seed']}  {f['reason']}  t={f['time']:.2f}s  plans={f['plans']}")

    if not failures:
        print("Tidak ada kegagalan. Selesai.")
        return

    # Fase 2: replay kegagalan secara visual, satu per satu
    env.render = True
    pygame.init()
    env.screen = pygame.display.set_mode(
        (int(env.field.length), int(env.field.width))
    )
    pygame.display.set_caption("Failure Inspector")
    env.clock = pygame.time.Clock()
    env.font = pygame.font.SysFont("monospace", 16)

    from environment.assets import load_sprite
    from environment.robot import RobotBody
    env.robot.sprite = load_sprite("team.png", int(env.robot.radius * 2))
    env.enemy_sprite = load_sprite("enemy.png", 36)
    env.ball.sprite = load_sprite("fifa-ball.png", int(env.ball.radius * 2))

    idx = 0
    while idx < len(failures):
        info = failures[idx]
        print(f"\n--- Kegagalan {idx+1}/{len(failures)}: "
              f"seed={info['seed']} {info['reason']} ---")

        env.reset(info["seed"])
        outcome = None
        paused_at_end = False

        # flag untuk keluar dari while episode
        advance = False
        quit_app = False

        while not advance and not quit_app:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    quit_app = True
                    break
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        quit_app = True
                        break
                    elif event.key == pygame.K_SPACE and paused_at_end:
                        advance = True
                        break
                    elif event.key == pygame.K_r:
                        env.reset(info["seed"])
                        outcome = None
                        paused_at_end = False

            if advance or quit_app:
                break

            if paused_at_end:
                env.draw()
                pygame.display.flip()
                env.clock.tick(30)
                continue

            if outcome is None:
                outcome = env.tick()
                env.draw()
                pygame.display.flip()
                env.clock.tick(60)
                if outcome is not None:
                    if outcome == "success":
                        env.show_banner("SUCCESS", (80, 220, 90))
                    elif outcome == "collision":
                        env.show_banner("COLLISION", (220, 60, 60))
                    else:
                        env.show_banner("TIMEOUT", (220, 180, 60))
                    paused_at_end = True

        if quit_app:
            break
        idx += 1

    pygame.quit()
    print("Selesai.")


if __name__ == "__main__":
    sys.exit(main())