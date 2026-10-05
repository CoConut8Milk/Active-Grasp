"""DDQN training loop with curriculum learning."""

import argparse
import csv
import os
import random

import numpy as np
import rclpy
from rclpy.node import Node
import torch

from ag_agent.env import ActiveGraspEnv
from ag_agent.agent import DQNAgent
from ag_agent.replay import ReplayBuffer


def declare(node):
    defaults = {
        "grid_height": 48, "grid_width": 48, "n_dirs": 8, "n_views": 4,
        "z_max": 0.35, "max_steps": 25, "lr": 3e-4, "gamma": 0.9,
        "batch_size": 64, "capacity": 20000, "target_update": 300,
        "in_channels": 6, "eps_start": 1.0, "eps_end": 0.05,
        "eps_decay_episodes": 300, "min_objects": 2, "max_objects": 8,
        "curriculum_window": 20, "curriculum_up": 0.70,
        "curriculum_down": 0.30, "episodes": 400, "save_every": 50,
        "log_every": 5, "seed": 0,
        "checkpoint_dir": "checkpoints", "results_dir": "results",
    }
    for name, value in defaults.items():
        node.declare_parameter(name, value)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--resume", default=None, help="checkpoint to resume")
    parser.add_argument("--episodes", type=int, default=None)
    args = parser.parse_args()

    rclpy.init()
    node = Node("ag_train")
    declare(node)
    p = node.get_parameter

    height, width = p("grid_height").value, p("grid_width").value
    env = ActiveGraspEnv(
        node, height=height, width=width,
        n_dirs=p("n_dirs").value, n_views=p("n_views").value,
        max_steps=p("max_steps").value, z_max=p("z_max").value,
    )
    agent = DQNAgent(
        env.action_space,
        lr=p("lr").value, gamma=p("gamma").value,
        in_channels=p("in_channels").value, seed=p("seed").value,
    )
    replay = ReplayBuffer(
        p("capacity").value, (p("in_channels").value, height, width),
        seed=p("seed").value,
    )

    n_episodes = args.episodes if args.episodes is not None else p("episodes").value
    if args.resume:
        agent.load(args.resume)
        node.get_logger().info(f"resumed from {args.resume}")

    os.makedirs(p("checkpoint_dir").value, exist_ok=True)
    os.makedirs(p("results_dir").value, exist_ok=True)
    csv_path = os.path.join(p("results_dir").value, "training_log.csv")
    csv_file = open(csv_path, "a", newline="")
    writer = csv.writer(csv_file)
    if os.path.getsize(csv_path) == 0:
        writer.writerow([
            "episode", "objects", "cleared", "clearance", "steps", "reward",
            "views", "epsilon", "loss",
        ])

    torch.manual_seed(p("seed").value)
    random.seed(p("seed").value)
    np.random.seed(p("seed").value)

    n_objects = p("min_objects").value
    history = []
    total_steps = 0
    last_loss = float("nan")

    for episode in range(n_episodes):
        state = env.reset(n_objects)
        done = False
        ep_reward = 0.0
        ep_views = 0
        epsilon = max(
            p("eps_end").value,
            p("eps_start").value
            - (p("eps_start").value - p("eps_end").value)
            * episode / p("eps_decay_episodes").value,
        )

        while not done:
            mask = env.valid_mask()
            action = agent.select(env.state_tensor(), mask, epsilon)
            next_state, reward, done, info = env.step(action)
            replay.push(
                env.state_tensor(state), action, reward,
                env.state_tensor(next_state), done,
            )
            state = next_state
            ep_reward += reward
            ep_views += info["kind"] == "view"
            total_steps += 1

            if len(replay) >= p("batch_size").value:
                last_loss = agent.update(
                    replay.sample(p("batch_size").value)
                )
                if total_steps % p("target_update").value == 0:
                    agent.sync()

        clearance = env.cleared / max(env.n_objects, 1)
        history.append(clearance)
        if len(history) > p("curriculum_window").value:
            history.pop(0)
        if len(history) == p("curriculum_window").value:
            avg = np.mean(history)
            if avg >= p("curriculum_up").value:
                n_objects = min(n_objects + 1, p("max_objects").value)
                history.clear()
            elif avg <= p("curriculum_down").value:
                n_objects = max(n_objects - 1, p("min_objects").value)
                history.clear()

        writer.writerow([
            episode, env.n_objects, env.cleared, round(clearance, 3),
            env.steps, round(ep_reward, 3), ep_views, round(epsilon, 3),
            round(last_loss, 5) if np.isfinite(last_loss) else "",
        ])
        csv_file.flush()

        if episode % p("log_every").value == 0:
            node.get_logger().info(
                f"ep {episode}: objects={env.n_objects} "
                f"clearance={clearance:.2f} steps={env.steps} "
                f"reward={ep_reward:.2f} views={ep_views} eps={epsilon:.2f}"
            )
        if (episode + 1) % p("save_every").value == 0:
            path = os.path.join(p("checkpoint_dir").value, f"agent_{episode + 1}.pth")
            agent.save(path)
            node.get_logger().info(f"saved {path}")

    final_path = os.path.join(p("checkpoint_dir").value, "agent_final.pth")
    agent.save(final_path)
    csv_file.close()
    node.get_logger().info(f"training finished; final checkpoint at {final_path}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

