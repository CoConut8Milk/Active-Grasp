"""Discrete action space: grasp pixel, push pixel+direction, choose viewpoint."""

import numpy as np


class ActionSpace:
    def __init__(self, height, width, n_dirs=8, n_views=4, border=3):
        self.height = int(height)
        self.width = int(width)
        self.n_dirs = int(n_dirs)
        self.n_views = int(n_views)
        # Depth cameras are unreliable at the very border of the image, which
        # shows up as a permanent "tall object" in the raster (the same corner
        # cell in every episode). Never grasp there. Kept small for tiny test
        # grids so that they still have valid interior cells.
        self.border = max(0, min(int(border), (min(self.height, self.width) - 1) // 2))
        self.grasp_size = self.height * self.width
        self.push_size = self.grasp_size * self.n_dirs
        self.view_size = self.n_views
        self.total = self.grasp_size + self.push_size + self.view_size

    def decode(self, action):
        a = int(action)
        if a < self.grasp_size:
            return "grasp", a % self.width, a // self.width
        a -= self.grasp_size
        if a < self.push_size:
            # Direction-major layout, matching the flattened push head:
            # [dir0 map, dir1 map, ...] each of H*W cells.
            d = a // self.grasp_size
            p = a % self.grasp_size
            return "push", p % self.width, p // self.width, d
        return "view", a - self.push_size

    def encode_grasp(self, u, v):
        return v * self.width + u

    def encode_push(self, u, v, direction):
        return (
            self.grasp_size
            + direction * self.grasp_size
            + (v * self.width + u)
        )

    def encode_view(self, view_id):
        return self.grasp_size + self.push_size + view_id

    def valid_mask(self, state6):
        """state6: (6, H, W) with channel 5 = occupancy mask (0..1)."""
        interior = np.zeros((self.height, self.width), dtype=bool)
        if self.border > 0:
            interior[
                self.border: self.height - self.border,
                self.border: self.width - self.border,
            ] = True
        else:
            interior[:] = True
        occupied = ((state6[5] > 0.5) & interior).reshape(-1)
        mask = np.zeros(self.total, dtype=bool)
        mask[: self.grasp_size] = occupied
        mask[self.grasp_size: self.grasp_size + self.push_size] = np.tile(
            occupied, self.n_dirs
        )
        mask[self.grasp_size + self.push_size:] = True
        return mask
