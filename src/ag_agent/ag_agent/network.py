"""Fully-convolutional Q-network: pixelwise grasp/push Q maps + view head."""

import torch
import torch.nn as nn
import torch.nn.functional as F


class ActiveGraspNet(nn.Module):
    def __init__(self, in_channels=6, n_dirs=8, n_views=4, base_channels=64):
        super().__init__()
        c = base_channels
        self.backbone = nn.Sequential(
            nn.Conv2d(in_channels, c, 3, padding=1),
            nn.ReLU(inplace=True),
            nn.Conv2d(c, c, 3, padding=2, dilation=2),
            nn.ReLU(inplace=True),
            nn.Conv2d(c, c, 3, padding=4, dilation=4),
            nn.ReLU(inplace=True),
            nn.Conv2d(c, c, 3, padding=8, dilation=8),
            nn.ReLU(inplace=True),
        )
        self.grasp_head = nn.Conv2d(c, 1, 1)
        self.push_head = nn.Conv2d(c, n_dirs, 1)
        self.view_head = nn.Sequential(
            nn.AdaptiveAvgPool2d(1), nn.Flatten(), nn.Linear(c, n_views)
        )

    def forward(self, x):
        f = self.backbone(x)
        g = self.grasp_head(f).flatten(1)
        p = self.push_head(f).flatten(2).flatten(1)
        v = self.view_head(f)
        return torch.cat([g, p, v], dim=1)


def masked_argmax(q_values, mask):
    q = q_values.clone()
    q[~mask] = float("-inf")
    return q.argmax(dim=1)

