"""Double DQN training loop primitives."""

import random

import numpy as np
import torch
import torch.nn.functional as F

from ag_agent.network import ActiveGraspNet, masked_argmax


class DQNAgent:
    def __init__(self, action_space, device="cpu", lr=3e-4, gamma=0.9,
                 in_channels=6, seed=0):
        self.action_space = action_space
        self.device = torch.device(device)
        self.gamma = gamma
        self._rng = random.Random(seed)

        self.online = ActiveGraspNet(
            in_channels=in_channels,
            n_dirs=action_space.n_dirs,
            n_views=action_space.n_views,
        ).to(self.device)
        self.target = ActiveGraspNet(
            in_channels=in_channels,
            n_dirs=action_space.n_dirs,
            n_views=action_space.n_views,
        ).to(self.device)
        self.target.load_state_dict(self.online.state_dict())
        self.optimizer = torch.optim.Adam(self.online.parameters(), lr=lr)

    def _to_tensor(self, state):
        return torch.from_numpy(np.asarray(state, dtype=np.float32)).unsqueeze(0).to(
            self.device
        )

    def select(self, state, mask, epsilon, eval_mode=False):
        if not eval_mode and self._rng.random() < epsilon:
            candidates = np.flatnonzero(mask)
            return int(self._rng.choice(candidates))
        with torch.no_grad():
            q = self.online(self._to_tensor(state))
            mask_t = torch.from_numpy(np.asarray(mask)).unsqueeze(0).to(self.device)
            return int(masked_argmax(q, mask_t).item())

    def update(self, batch):
        states = torch.from_numpy(batch["states"]).to(self.device)
        actions = torch.from_numpy(batch["actions"]).unsqueeze(1).to(self.device)
        rewards = torch.from_numpy(batch["rewards"]).unsqueeze(1).to(self.device)
        next_states = torch.from_numpy(batch["next_states"]).to(self.device)
        dones = torch.from_numpy(batch["dones"].astype(np.float32)).unsqueeze(1).to(
            self.device
        )

        q = self.online(states).gather(1, actions)
        with torch.no_grad():
            next_masks = []
            for i in range(next_states.shape[0]):
                next_masks.append(
                    self.action_space.valid_mask(next_states[i].cpu().numpy())
                )
            next_mask = torch.from_numpy(np.stack(next_masks)).to(self.device)
            q_next_online = self.online(next_states)
            best = masked_argmax(q_next_online, next_mask).unsqueeze(1)
            q_next_target = self.target(next_states).gather(1, best)
            target = rewards + self.gamma * (1.0 - dones) * q_next_target

        loss = F.smooth_l1_loss(q, target)
        self.optimizer.zero_grad()
        loss.backward()
        torch.nn.utils.clip_grad_norm_(self.online.parameters(), 10.0)
        self.optimizer.step()
        return float(loss.item())

    def sync(self):
        self.target.load_state_dict(self.online.state_dict())

    def save(self, path):
        torch.save(
            {
                "online": self.online.state_dict(),
                "optimizer": self.optimizer.state_dict(),
            },
            path,
        )

    def load(self, path):
        ckpt = torch.load(path, map_location=self.device)
        self.online.load_state_dict(ckpt["online"])
        self.target.load_state_dict(ckpt["online"])
        if "optimizer" in ckpt:
            self.optimizer.load_state_dict(ckpt["optimizer"])

