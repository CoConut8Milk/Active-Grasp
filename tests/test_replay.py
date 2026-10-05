import numpy as np

from ag_agent.replay import ReplayBuffer


def test_replay_buffer_wraps_and_samples():
    buffer = ReplayBuffer(4, (3, 2, 2), seed=0)
    for i in range(6):
        buffer.push(
            np.full((3, 2, 2), i, dtype=np.float32),
            i,
            float(i),
            np.full((3, 2, 2), i + 1, dtype=np.float32),
            i == 5,
        )
    assert len(buffer) == 4
    batch = buffer.sample(3)
    assert batch["states"].shape == (3, 3, 2, 2)
    assert batch["dones"].dtype == bool
    # The oldest transition (i=0,1) was overwritten.
    assert batch["actions"].min() >= 2

