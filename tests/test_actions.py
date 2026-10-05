import numpy as np

from ag_agent.actions import ActionSpace


def test_encode_decode_round_trip():
    space = ActionSpace(48, 48, n_dirs=8, n_views=4)
    for action in range(space.total):
        kind, *payload = space.decode(action)
        if kind == "grasp":
            assert space.encode_grasp(*payload) == action
        elif kind == "push":
            assert space.encode_push(*payload) == action
        else:
            assert space.encode_view(*payload) == action


def test_valid_mask():
    space = ActionSpace(4, 4, n_dirs=2, n_views=2)
    state = np.zeros((6, 4, 4), dtype=np.float32)
    state[5, 1, 1] = 1.0
    mask = space.valid_mask(state)
    assert mask.sum() == 1 + 2 + 2  # one grasp cell, 2 push dirs, 2 views
    assert mask[space.encode_grasp(1, 1)]
    assert mask[space.encode_push(1, 1, 0)]
    assert mask[space.encode_push(1, 1, 1)]
    assert mask[space.encode_view(0)]
    assert not mask[space.encode_grasp(0, 0)]

