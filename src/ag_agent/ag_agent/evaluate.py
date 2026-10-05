"""Evaluate a trained policy over a sweep of clutter densities."""

import argparse
import json
import os

import numpy as np
import rclpy
from rclpy.node import Node

from ag_agent.env import ActiveGraspEnv
from ag_agent.agent import DQNAgent


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--episodes", type=int, default=10)
    parser.add_argument("--objects", type=int, nargs="+", default=[2, 4, 6, 8])
    parser.add_argument("--epsilon", type=float, default=0.05)
    parser.add_argument("--out", default="results/evaluation.json")
    args = parser.parse_args()

    rclpy.init()
    node = Node("ag_evaluate")
    env = ActiveGraspEnv(node, height=48, width=48, n_dirs=8, n_views=4)
    agent = DQNAgent(env.action_space)
    agent.load(args.checkpoint)

    results = {}
    for n in args.objects:
        clearances, steps_list, view_rates = [], [], []
        for _ in range(args.episodes):
            state = env.reset(n)
            done = False
            views = 0
            while not done:
                mask = env.valid_mask()
                action = agent.select(
                    env.state_tensor(), mask, args.epsilon, eval_mode=True
                )
                state, reward, done, info = env.step(action)
                views += info["kind"] == "view"
            clearances.append(env.cleared / max(env.n_objects, 1))
            steps_list.append(env.steps)
            view_rates.append(views / max(env.steps, 1))
        results[str(n)] = {
            "clearance_mean": float(np.mean(clearances)),
            "clearance_std": float(np.std(clearances)),
            "steps_mean": float(np.mean(steps_list)),
            "view_rate_mean": float(np.mean(view_rates)),
        }

    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    with open(args.out, "w") as f:
        json.dump(results, f, indent=2)

    print("objects | clearance (mean±std) | steps | view rate")
    for n, r in results.items():
        print(
            f"{n:>7} | {r['clearance_mean']:.2f}±{r['clearance_std']:.2f}"
            f"     | {r['steps_mean']:.1f} | {r['view_rate_mean']:.2f}"
        )
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

