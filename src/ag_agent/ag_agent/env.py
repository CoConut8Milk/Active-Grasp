"""ROS service-backed environment (no external gym dependency)."""

import numpy as np
import time

from std_srvs.srv import Trigger
from ag_interfaces.srv import Observe, Grasp, Push, MoveView, Reset
from ag_agent.actions import ActionSpace


def _decode_image(msg):
    dtype = np.float32 if "32FC1" in msg.encoding else np.uint8
    arr = np.frombuffer(msg.data, dtype=dtype)
    if len(arr) < msg.height * msg.width:
        return np.zeros((msg.height, msg.width), dtype=dtype)
    if dtype == np.float32:
        return arr[: msg.height * msg.width].reshape(msg.height, msg.width)
    step = max(msg.step // np.dtype(np.uint8).itemsize, msg.width * 3)
    arr = arr.reshape(msg.height, step)
    if "mono8" in msg.encoding:
        return arr[:, : msg.width]
    return arr[:, : msg.width * 3].reshape(msg.height, msg.width, 3)


class ActiveGraspEnv:
    def __init__(self, node, height=48, width=48, n_dirs=8, n_views=4,
                 max_steps=25, z_max=0.35):
        self.node = node
        self.action_space = ActionSpace(height, width, n_dirs, n_views)
        self.max_steps = max_steps
        self.z_max = z_max
        self.n_objects = 0
        self.cleared = 0
        self.steps = 0
        self.last_state = None

        self._observe = node.create_client(Observe, "/perception_node/observe")
        self._grasp = node.create_client(Grasp, "/execution_node/grasp")
        self._push = node.create_client(Push, "/execution_node/push")
        self._view = node.create_client(MoveView, "/execution_node/move_view")
        self._reset = node.create_client(Reset, "/world_manager/reset_scene")
        self._clear = node.create_client(Trigger, "/perception_node/clear")
        self._clear_aux = node.create_client(Trigger, "/perception_node/clear_aux")

        clients = [self._observe, self._grasp, self._push, self._view,
                   self._reset, self._clear, self._clear_aux]
        for cli in clients:
            # Gazebo 启动较慢时服务会晚到，这里一直等（每 10 秒提示一次）。
            import rclpy
            while not cli.wait_for_service(timeout_sec=10.0):
                if not rclpy.ok():
                    raise RuntimeError("ROS 已关闭，停止等待服务")
                node.get_logger().info(f"waiting for service {cli.srv_name} ...")

    def _call(self, client, request, timeout=30.0):
        future = client.call_async(request)
        import rclpy
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
        if not future.done() or future.result() is None:
            raise TimeoutError(f"service {client.srv_name} timed out")
        return future.result()

    def _trigger(self, client):
        req = Trigger.Request()
        return self._call(client, req)

    def observe(self, auxiliary=False):
        req = Observe.Request()
        req.auxiliary_view = bool(auxiliary)
        # Wait until perception has calibrated and returns real data: on a
        # fresh launch the arm first drives home and the camera calibration
        # only starts once the robot stands still.
        deadline = time.monotonic() + 120.0
        attempts = 0
        while True:
            rsp = self._call(self._observe, req)
            if len(rsp.height.data) > 0 or time.monotonic() > deadline:
                break
            attempts += 1
            if attempts % 10 == 1:
                self.node.get_logger().info(
                    "waiting for perception to finish camera calibration ..."
                )
            time.sleep(0.5)
        if len(rsp.height.data) == 0:
            raise RuntimeError(
                "perception never produced a state; check the perception_node "
                "log for 'not calibrated yet: ...' messages"
            )
        height = _decode_image(rsp.height)
        uncertainty = _decode_image(rsp.uncertainty)
        color = _decode_image(rsp.color).astype(np.float32)
        mask = _decode_image(rsp.mask).astype(np.float32) / 255.0
        return np.stack([height, uncertainty, color[:, :, 0], color[:, :, 1],
                         color[:, :, 2], mask])

    def reset(self, num_objects, randomize=True):
        self._trigger(self._clear)
        req = Reset.Request()
        req.num_objects = int(num_objects)
        req.randomize_objects = bool(randomize)
        rsp = self._call(self._reset, req)
        self.n_objects = int(rsp.objects_spawned)
        self.cleared = 0
        self.steps = 0
        self.last_state = self.observe()
        return self.last_state

    def state_tensor(self, state=None):
        s = state if state is not None else self.last_state
        out = s.copy()
        out[0] = np.clip(out[0] / self.z_max, 0.0, 1.0)
        out[2:6] = out[2:6] / 255.0
        return out

    def valid_mask(self, state=None):
        s = state if state is not None else self.last_state
        return self.action_space.valid_mask(s)

    def step(self, action):
        kind, *payload = self.action_space.decode(action)
        info = {"kind": kind}

        if kind == "grasp":
            u, v = payload
            req = Grasp.Request()
            req.u, req.v = u, v
            req.height = float(self.last_state[0, v, u])
            rsp = self._call(self._grasp, req)
            self._trigger(self._clear_aux)
            next_state = self.observe()
            success = bool(rsp.success)
            if success:
                self.cleared += 1
            reward = 10.0 if success else -0.3
            info["success"] = success

        elif kind == "push":
            u, v, direction = payload
            before = self.last_state[0]
            req = Push.Request()
            req.u, req.v = u, v
            req.direction = direction
            req.height = float(self.last_state[0, v, u])
            rsp = self._call(self._push, req)
            self._trigger(self._clear_aux)
            next_state = self.observe()
            delta = float(np.mean(np.abs(next_state[0] - before)))
            reward = float(np.clip(30.0 * delta - 0.1, -0.2, 1.0))
            info["delta"] = delta

        else:
            view_id = payload[0]
            before_u = float(np.mean(self.last_state[1]))
            req = MoveView.Request()
            req.view_id = view_id
            rsp = self._call(self._view, req)
            next_state = self.observe()
            after_u = float(np.mean(next_state[1]))
            reward = float(np.clip(2.0 * (before_u - after_u) - 0.05, -0.05, 1.0))
            info["du"] = before_u - after_u

        self.steps += 1
        done = self.cleared >= self.n_objects or self.steps >= self.max_steps
        if done and self.cleared >= self.n_objects:
            reward += 5.0
        self.last_state = next_state
        info.update({"cleared": self.cleared, "steps": self.steps})
        return next_state, float(reward), bool(done), info
