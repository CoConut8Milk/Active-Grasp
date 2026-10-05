import pathlib

import numpy as np

from ag_execution.kinematics import (
    RobotModel,
    look_at_matrix,
    pose_from_rot_trans,
)

URDF = (
    pathlib.Path(__file__).resolve().parents[1]
    / "src"
    / "ag_description"
    / "urdf"
    / "ag_arm.urdf"
).read_text()


def _model():
    return RobotModel(URDF)


def _pose(rotation, position):
    return pose_from_rot_trans(rotation, position)


def test_fk_chain_geometry():
    model = _model()
    t = model.fk(np.zeros(6))
    # Vertical chain: tool0 is the sum of all z-offsets above base.
    assert np.allclose(t["tool0"][:3, 3], [0.0, 0.0, 1.22], atol=1e-3)
    # Camera sits to the +x side, viewing along +z (chain direction).
    cam = t["camera_link"]
    assert cam[:3, 3][0] > 0.05
    assert np.allclose(cam[:3, :3] @ np.array([1.0, 0, 0]), [0, 0, 1], atol=1e-4)


def test_ik_grasp_points_across_workspace():
    model = _model()
    rng = np.random.default_rng(0)
    orient = np.diag([1.0, -1.0, -1.0])
    seeds = [np.zeros(6), [0, -0.6, 1.4, -0.8, 0, 0]]

    for x in np.linspace(0.24, 0.66, 6):
        for y in np.linspace(-0.20, 0.20, 5):
            target = _pose(orient, [x, y, 0.095])
            q = model.ik_retry(target, seeds, random_seed=int(x * 100 + y * 100))
            assert q is not None, f"IK failed at ({x:.2f}, {y:.2f})"
            t = model.fk(q)["tool0"]
            assert np.linalg.norm(t[:3, 3] - [x, y, 0.095]) < 0.02


def test_ik_viewpoints_and_home():
    model = _model()
    cam_tool = model.camera_in_tool()
    poses = [
        ([0.68, 0.0, 0.62], [0.45, 0.0, 0.02]),
        ([0.45, 0.00, 0.72], [0.45, 0.0, 0.02]),
        ([0.45, -0.28, 0.52], [0.45, 0.0, 0.02]),
        ([0.45, 0.28, 0.52], [0.45, 0.0, 0.02]),
        ([0.18, 0.00, 0.60], [0.45, 0.0, 0.02]),
    ]
    for pos, look in poses:
        cam_target = _pose(look_at_matrix(pos, look), pos)
        tool_target = cam_target @ cam_tool
        q = model.ik_retry(tool_target, [np.zeros(6)], random_seed=7)
        assert q is not None, f"viewpoint IK failed at {pos}"
        # The reconstructed camera must look at the workspace center.
        t_cam = model.fk(q)["camera_link"]
        view_dir = t_cam[:3, :3] @ np.array([1.0, 0, 0])
        expect = np.array(look) - np.array(pos)
        expect /= np.linalg.norm(expect)
        assert np.dot(view_dir, expect) > 0.999


def test_ik_bin_pose():
    model = _model()
    orient = np.diag([1.0, -1.0, -1.0])
    target = _pose(orient, [0.45, -0.34, 0.32])
    q = model.ik_retry(target, [np.zeros(6)], random_seed=7)
    assert q is not None
    t = model.fk(q)["tool0"]
    assert np.linalg.norm(t[:3, 3] - [0.45, -0.34, 0.32]) < 0.02


def test_ik_respects_joint_limits():
    model = _model()
    lower, upper = model.limits
    orient = np.diag([1.0, -1.0, -1.0])
    q = model.ik_retry(
        _pose(orient, [0.45, 0.0, 0.095]), [np.zeros(6)], random_seed=7
    )
    assert q is not None
    assert np.all(q >= lower - 1e-6)
    assert np.all(q <= upper + 1e-6)
