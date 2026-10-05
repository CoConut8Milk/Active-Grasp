#!/usr/bin/env python3
"""Plot training curves from results/training_log.csv."""

import argparse

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--csv", default="results/training_log.csv")
    parser.add_argument("--out", default="results/training_curves.png")
    args = parser.parse_args()

    data = np.genfromtxt(
        args.csv, delimiter=",", names=True, dtype=None, encoding="utf-8"
    )
    fig, axes = plt.subplots(2, 2, figsize=(12, 7))

    axes[0, 0].plot(data["episode"], data["clearance"], ".", alpha=0.5)
    axes[0, 0].set_title("Episode clearance")
    axes[0, 0].set_xlabel("episode")

    axes[0, 1].plot(data["episode"], data["objects"])
    axes[0, 1].set_title("Curriculum level (objects)")
    axes[0, 1].set_xlabel("episode")

    axes[1, 0].plot(data["episode"], data["reward"], ".", alpha=0.5)
    axes[1, 0].set_title("Episode reward")
    axes[1, 0].set_xlabel("episode")

    axes[1, 1].plot(data["episode"], data["views"])
    axes[1, 1].set_title("View actions per episode")
    axes[1, 1].set_xlabel("episode")

    fig.tight_layout()
    fig.savefig(args.out, dpi=120)
    print(f"wrote {args.out}")


if __name__ == "__main__":
    main()

