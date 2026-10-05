"""Hand-coded baselines used to validate the pipeline and for ablations."""

import numpy as np


class GreedyPolicy:
    """Grasp the tallest visible cell; push it toward the center on failure."""

    def __init__(self, action_space, max_failures=2, seed=0):
        self.action_space = action_space
        self.max_failures = max_failures
        self._rng = np.random.default_rng(seed)
        self._failures = {}

    def act(self, state, mask):
        h, w = state[0].shape
        height = state[0]
        occupied = mask[: self.action_space.grasp_size].reshape(h, w)
        if not occupied.any():
            # nothing visible: pick a view
            return self.action_space.encode_view(
                self._rng.integers(self.action_space.n_views)
            )
        ys, xs = np.nonzero(occupied)
        best = int(np.argmax(height[ys, xs]))
        v, u = ys[best], xs[best]

        if self._failures.get((u, v), 0) < self.max_failures:
            return self.action_space.encode_grasp(u, v)

        # Push the stubborn cell toward the workspace center.
        cx, cy = (w - 1) / 2.0, (h - 1) / 2.0
        direction = int(round(np.arctan2(cy - v, cx - u) / (np.pi / 4.0))) % 8
        return self.action_space.encode_push(u, v, direction)

    def observe_result(self, action, success):
        kind, *payload = self.action_space.decode(action)
        if kind == "grasp":
            u, v = payload
            if success:
                self._failures.pop((u, v), None)
            else:
                self._failures[(u, v)] = self._failures.get((u, v), 0) + 1


class RandomPolicy:
    def __init__(self, action_space, seed=0):
        self.action_space = action_space
        self._rng = np.random.default_rng(seed)

    def act(self, state, mask):
        idx = np.flatnonzero(mask)
        return int(self._rng.choice(idx))

    def observe_result(self, action, success):
        pass

