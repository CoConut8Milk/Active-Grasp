"""URDF-driven forward kinematics and numeric inverse kinematics.

Parses the ag_arm URDF at runtime so the kinematic model can never drift from
the simulated robot. FK follows the link tree from base_link; IK uses a
trust-region least-squares solve on the 6D pose error, seeded from the current
configuration, which is fast and reliable for waypoint-style motion.
"""

import xml.etree.ElementTree as ET

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


def rpy_to_matrix(roll, pitch, yaw):
    """URDF rpy convention: R = Rz(yaw) Ry(pitch) Rx(roll)."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def axis_rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis = axis / (np.linalg.norm(axis) + 1e-12)
    k = np.array(
        [[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]]
    )
    return np.eye(3) + np.sin(angle) * k + (1 - np.cos(angle)) * (k @ k)


def look_at_matrix(position, target):
    """Camera link frame with +x toward the target (Gazebo view convention)."""
    x = np.asarray(target, dtype=float) - np.asarray(position, dtype=float)
    x = x / (np.linalg.norm(x) + 1e-12)
    up = np.array([0.0, 0.0, 1.0])
    y = np.cross(up, x)
    if np.linalg.norm(y) < 1e-6:
        y = np.array([0.0, 1.0, 0.0])
    y = y / np.linalg.norm(y)
    z = np.cross(x, y)
    return np.column_stack([x, y, z])


def pose_from_rot_trans(rotation, translation):
    t = np.eye(4)
    t[:3, :3] = np.asarray(rotation)
    t[:3, 3] = np.asarray(translation)
    return t


class RobotModel:
    """Minimal URDF kinematic model for serial arms with fixed grippers."""

    def __init__(self, urdf_text, base_link="base_link"):
        root = ET.fromstring(urdf_text)
        self._joints = {}
        self._children = {}

        for joint in root.findall("joint"):
            name = joint.attrib["name"]
            jtype = joint.attrib["type"]
            parent = joint.find("parent").attrib["link"]
            child = joint.find("child").attrib["link"]
            origin = joint.find("origin")
            xyz = [0.0, 0.0, 0.0]
            rpy = [0.0, 0.0, 0.0]
            if origin is not None:
                if "xyz" in origin.attrib:
                    xyz = [float(v) for v in origin.attrib["xyz"].split()]
                if "rpy" in origin.attrib:
                    rpy = [float(v) for v in origin.attrib["rpy"].split()]
            axis_el = joint.find("axis")
            axis = [0.0, 0.0, 1.0]
            if axis_el is not None:
                axis = [float(v) for v in axis_el.attrib["xyz"].split()]
            limit = joint.find("limit")
            lower, upper = None, None
            if limit is not None:
                if "lower" in limit.attrib:
                    lower = float(limit.attrib["lower"])
                if "upper" in limit.attrib:
                    upper = float(limit.attrib["upper"])

            self._joints[name] = {
                "type": jtype,
                "parent": parent,
                "child": child,
                "xyz": xyz,
                "rpy": rpy,
                "axis": axis,
                "lower": lower,
                "upper": upper,
            }
            self._children.setdefault(parent, []).append(name)

        self.base_link = base_link
        self._order = self._bfs_order()
        self.arm_joints = [
            name
            for name in self._order
            if self._joints[name]["type"] in ("revolute", "continuous")
        ]
        self.gripper_joints = [
            name for name in self._order if self._joints[name]["type"] == "prismatic"
        ]
        if len(self.arm_joints) != 6:
            raise ValueError(
                f"expected 6 revolute arm joints, found {len(self.arm_joints)}"
            )

    def _bfs_order(self):
        order = []
        queue = [self.base_link]
        visited = {self.base_link}
        while queue:
            parent = queue.pop(0)
            for joint in self._children.get(parent, []):
                child = self._joints[joint]["child"]
                order.append(joint)
                if child not in visited:
                    visited.add(child)
                    queue.append(child)
        return order

    @property
    def limits(self):
        lower, upper = [], []
        for name in self.arm_joints:
            j = self._joints[name]
            lo = j["lower"] if j["lower"] is not None else -3.1416
            hi = j["upper"] if j["upper"] is not None else 3.1416
            lower.append(lo)
            upper.append(hi)
        return np.array(lower), np.array(upper)

    def fk(self, q, extra_joints=None):
        """Forward kinematics; returns {link: 4x4 base->link} transforms."""
        q = np.asarray(q, dtype=float)
        if q.shape != (6,):
            raise ValueError("q must have 6 arm joint values")
        values = {name: float(v) for name, v in zip(self.arm_joints, q)}
        if extra_joints:
            values.update({k: float(v) for k, v in extra_joints.items()})

        transforms = {self.base_link: np.eye(4)}
        for joint_name in self._order:
            joint = self._joints[joint_name]
            parent_t = transforms[joint["parent"]]
            origin = pose_from_rot_trans(
                rpy_to_matrix(*joint["rpy"]), joint["xyz"]
            )
            motion = np.eye(4)
            jtype = joint["type"]
            if jtype == "fixed":
                pass
            elif jtype in ("revolute", "continuous"):
                motion[:3, :3] = axis_rotation(joint["axis"], values[joint_name])
            elif jtype == "prismatic":
                motion[:3, 3] = np.asarray(joint["axis"]) * values.get(joint_name, 0.0)
            transforms[joint["child"]] = parent_t @ origin @ motion
        return transforms

    def camera_in_tool(self):
        """Fixed camera->tool transform (maps camera coords into tool coords)."""
        zero = self.fk(np.zeros(6))
        t_cam = zero["camera_link"]
        t_tool = zero["tool0"]
        return np.linalg.inv(t_cam) @ t_tool

    def ik(self, target_t, seed, link="tool0", pos_weight=1.0, ori_weight=0.35,
           max_nfev=150):
        """Solve IK for the 6D pose of `link`. Returns joints or None."""
        target_t = np.asarray(target_t, dtype=float)
        lower, upper = self.limits
        target_p = target_t[:3, 3]
        target_r = target_t[:3, :3]

        def residual(q):
            t = self.fk(q)[link]
            p_err = target_p - t[:3, 3]
            r_err = target_r.T @ t[:3, :3]
            rvec = Rotation.from_matrix(r_err).as_rotvec()
            return np.concatenate([p_err, ori_weight * rvec])

        result = least_squares(
            residual,
            x0=np.asarray(seed, dtype=float),
            bounds=(lower, upper),
            method="trf",
            max_nfev=max_nfev,
        )
        t = self.fk(result.x)[link]
        pos_err = float(np.linalg.norm(target_p - t[:3, 3]))
        rot_err = float(
            np.linalg.norm(Rotation.from_matrix(target_r.T @ t[:3, :3]).as_rotvec())
        )
        if pos_err > 0.02 or rot_err > 0.12:
            return None
        return result.x

    def ik_retry(self, target_t, seeds, link="tool0", n_random=8, random_seed=0, **kwargs):
        lower, upper = self.limits
        rng = np.random.default_rng(random_seed)
        for seed in seeds:
            q = self.ik(target_t, seed, link=link, **kwargs)
            if q is not None:
                return q
        for _ in range(int(n_random)):
            seed = lower + (upper - lower) * rng.random(6)
            q = self.ik(target_t, seed, link=link, **kwargs)
            if q is not None:
                return q
        return None
