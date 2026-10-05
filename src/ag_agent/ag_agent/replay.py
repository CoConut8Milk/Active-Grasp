"""Simple numpy replay buffer (torch-free so it can be unit-tested)."""

import numpy as np


class ReplayBuffer:
    def __init__(self, capacity, state_shape, seed=0):
        self.capacity = int(capacity)
        self._rng = np.random.default_rng(seed)
        self._states = np.zeros((self.capacity, *state_shape), dtype=np.float16)
        self._next_states = np.zeros((self.capacity, *state_shape), dtype=np.float16)
        self._actions = np.zeros(self.capacity, dtype=np.int64)
        self._rewards = np.zeros(self.capacity, dtype=np.float32)
        self._dones = np.zeros(self.capacity, dtype=bool)
        self._idx = 0
        self._size = 0

    def __len__(self):
        return self._size

    def push(self, state, action, reward, next_state, done):
        i = self._idx
        self._states[i] = state
        self._next_states[i] = next_state
        self._actions[i] = action
        self._rewards[i] = reward
        self._dones[i] = done
        self._idx = (self._idx + 1) % self.capacity
        self._size = min(self._size + 1, self.capacity)

    def sample(self, batch):
        n = min(batch, self._size)
        idx = self._rng.choice(self._size, n, replace=False)
        return {
            "states": self._states[idx].astype(np.float32),
            "actions": self._actions[idx],
            "rewards": self._rewards[idx],
            "next_states": self._next_states[idx].astype(np.float32),
            "dones": self._dones[idx],
        }

