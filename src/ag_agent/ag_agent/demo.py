"""Realtime demo: run the greedy baseline or a trained policy in the GUI."""

import argparse
import time

import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args

from ag_agent.env import ActiveGraspEnv
from ag_agent.agent import DQNAgent
from ag_agent.baselines import GreedyPolicy


def describe(action_space, action):
    kind, *payload = action_space.decode(action)
    if kind == "grasp":
        return f"GRASP cell ({payload[0]}, {payload[1]})"
    if kind == "push":
        return f"PUSH cell ({payload[0]}, {payload[1]}) dir {payload[2]}"
    return f"VIEW {payload[0]}"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--checkpoint", default=None)
    parser.add_argument("--objects", type=int, default=5)
    parser.add_argument("--epsilon", type=float, default=0.05)
    parser.add_argument("--step-delay", type=float, default=0.5)
    args = parser.parse_args(remove_ros_args()[1:])

    rclpy.init()
    node = Node("ag_demo")
    env = ActiveGraspEnv(node, height=48, width=48, n_dirs=8, n_views=4)

    if args.checkpoint:
        agent = DQNAgent(env.action_space)
        agent.load(args.checkpoint)
    else:
        agent = GreedyPolicy(env.action_space)

    state = env.reset(args.objects)
    done = False
    while not done:
        mask = env.valid_mask()
        if hasattr(agent, "select"):
            action = agent.select(env.state_tensor(), mask, args.epsilon, eval_mode=True)
        else:
            action = agent.act(state, mask)
        print(f"[demo] {describe(env.action_space, action)}")
        next_state, reward, done, info = env.step(action)
        if hasattr(agent, "observe_result"):
            success = info.get("success", False)
            agent.observe_result(action, success)
        print(
            f"[demo] reward={reward:.2f} cleared={info['cleared']}/{env.n_objects}"
        )
        state = next_state
        time.sleep(args.step_delay)

    print(f"[demo] episode done: cleared {env.cleared}/{env.n_objects}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

