"""ROS node that turns Grasp/Push/MoveView requests into arm motion."""

import math
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Bool
from std_srvs.srv import Trigger

from ag_interfaces.srv import Grasp, Push, MoveView, Observe
from ag_execution.kinematics import (
    RobotModel, look_at_matrix, pose_from_rot_trans,
)
from ag_execution.primitives import (
    grasp_plan, push_plan, view_plan, grid_to_xy, interpolate_joints,
)


class ExecutionNode(Node):
    def __init__(self):
        super().__init__("execution_node")

        self.declare_parameter("urdf_file", "")
        self.declare_parameter("arm_action", "/arm_controller/follow_joint_trajectory")
        self.declare_parameter("gripper_topic", "/gripper_controller/commands")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("observe_service", "/perception_node/observe")
        self.declare_parameter("base_link", "base_link")
        self.declare_parameter("tool_link", "tool0")
        self.declare_parameter("camera_link", "camera_link")
        self.declare_parameter("home_camera_pos", [0.68, 0.0, 0.62])
        self.declare_parameter("home_camera_look", [0.45, 0.0, 0.02])
        self.declare_parameter("bin_pos", [0.45, -0.34, 0.32])
        # ROS 2 参数不支持“字典列表”，这里用扁平数组：
        # 每个视角 6 个数 = [pos_x, pos_y, pos_z, look_x, look_y, look_z]
        self.declare_parameter("views_flat", [
            0.45, 0.00, 0.72, 0.45, 0.0, 0.02,
            0.45, -0.28, 0.52, 0.45, 0.0, 0.02,
            0.45, 0.28, 0.52, 0.45, 0.0, 0.02,
            0.18, 0.00, 0.60, 0.45, 0.0, 0.02,
        ])
        self.declare_parameter("grid_x_min", 0.20)
        self.declare_parameter("grid_x_max", 0.70)
        self.declare_parameter("grid_y_min", -0.25)
        self.declare_parameter("grid_y_max", 0.25)
        self.declare_parameter("grid_width", 48)
        self.declare_parameter("grid_height", 48)
        self.declare_parameter("min_grasp_height", 0.008)
        self.declare_parameter("gripper_close", [0.018, 0.018])
        self.declare_parameter("gripper_open", [0.0, 0.0])
        self.declare_parameter("held_q_sum", 0.033)
        self.declare_parameter("approach_offset", 0.18)
        self.declare_parameter("grasp_offset", 0.065)
        self.declare_parameter("lift_height", 0.30)
        self.declare_parameter("push_height", 0.075)
        self.declare_parameter("push_length", 0.09)
        self.declare_parameter("view_dwell", 1.2)
        self.declare_parameter("trajectory_steps", 8)
        # 同样用扁平数组：每个种子组 6 个关节角
        self.declare_parameter("seed_sets_flat", [
            0.0, -0.4, 1.3, -0.5, 0.0, 0.0,
            0.0, -0.8, 1.6, -0.8, 0.0, 0.0,
            0.5, -0.5, 1.2, -0.7, 0.0, 0.0,
            -0.5, -0.5, 1.2, -0.7, 0.0, 0.0,
            0.0, -1.0, 1.0, -0.9, 0.0, 0.0,
        ])

        urdf_file = self.get_parameter("urdf_file").value
        if not urdf_file:
            raise RuntimeError("urdf_file parameter is required")
        with open(urdf_file, "r") as f:
            self.model = RobotModel(f.read(), base_link=self.get_parameter("base_link").value)

        p = self.get_parameter
        self.tool_link = p("tool_link").value
        self.camera_link = p("camera_link").value
        views_flat = [float(v) for v in p("views_flat").value]
        views = [
            {
                "pos": views_flat[i * 6: i * 6 + 3],
                "look": views_flat[i * 6 + 3: i * 6 + 6],
            }
            for i in range(len(views_flat) // 6)
        ]
        self.grid = {
            "x_min": p("grid_x_min").value, "x_max": p("grid_x_max").value,
            "y_min": p("grid_y_min").value, "y_max": p("grid_y_max").value,
            "width": p("grid_width").value, "height": p("grid_height").value,
        }
        self.params = {
            "bin_pos": p("bin_pos").value,
            "views": views,
            "gripper_close": p("gripper_close").value,
            "gripper_open": p("gripper_open").value,
            "approach_offset": p("approach_offset").value,
            "grasp_offset": p("grasp_offset").value,
            "lift_height": p("lift_height").value,
            "push_height": p("push_height").value,
            "push_length": p("push_length").value,
            "view_dwell": p("view_dwell").value,
            "dur_approach": 0.9, "dur_descend": 0.55, "dur_close": 0.7,
            "dur_hold_check": 0.35, "dur_lift": 0.6, "dur_transit": 0.9,
            "dur_open": 0.5, "dur_home": 1.0, "dur_push": 0.7,
            "dur_retreat": 0.5, "dur_view": 1.0,
        }
        self.min_grasp_height = p("min_grasp_height").value
        self.held_q_sum = p("held_q_sum").value
        self.traj_steps = p("trajectory_steps").value
        seeds_flat = [float(v) for v in p("seed_sets_flat").value]
        self.seed_sets = [
            seeds_flat[i * 6: i * 6 + 6] for i in range(len(seeds_flat) // 6)
        ]

        self._joint_positions = {}
        self._current_q = np.zeros(6)
        self._have_state = False
        self.create_subscription(
            JointState, p("joint_states_topic").value, self._joint_cb,
            qos_profile_sensor_data,
        )

        ready_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._ready_pub = self.create_publisher(Bool, "/ag_execution/ready", ready_qos)
        self._gripper_pub = self.create_publisher(
            Float64MultiArray, p("gripper_topic").value, 10
        )
        self._arm_client = ActionClient(self, FollowJointTrajectory, p("arm_action").value)
        self._observe_cli = self.create_client(Observe, p("observe_service").value)

        self._grasp_srv = self.create_service(Grasp, "~/grasp", self._grasp_cb)
        self._push_srv = self.create_service(Push, "~/push", self._push_cb)
        self._view_srv = self.create_service(MoveView, "~/move_view", self._view_cb)
        self._home_srv = self.create_service(Trigger, "~/go_home", self._home_cb)

        self.get_logger().info("waiting for arm controller...")
        # Gazebo 在 WSL 里启动可能要一两分钟，这里一直等到服务出现为止。
        while rclpy.ok() and not self._arm_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().info(
                "still waiting for /arm_controller action server ..."
            )
        self._read_current_joints(timeout=5.0)

        self.home_q = self._solve_home()
        self.get_logger().info("moving to home pose...")
        if not self._move_to_joints(self.home_q, 2.0):
            raise RuntimeError("could not reach home pose at startup")
        self._ready_pub.publish(Bool(data=True))
        self.get_logger().info("execution ready")

    # ---------------- state helpers ----------------
    def _joint_cb(self, msg):
        self._joint_positions = dict(zip(msg.name, msg.position))
        if not self._have_state:
            try:
                q = np.array([self._joint_positions[n] for n in self.model.arm_joints])
                if q.shape == (6,):
                    self._current_q = q
                    self._have_state = True
            except KeyError:
                pass
        else:
            try:
                self._current_q = np.array(
                    [self._joint_positions[n] for n in self.model.arm_joints]
                )
            except KeyError:
                pass

    def _read_current_joints(self, timeout=5.0):
        while not self._have_state and timeout > 0:
            rclpy.spin_once(self, timeout_sec=0.05)
            timeout -= 0.05
        if not self._have_state:
            self.get_logger().warn("no joint state; assuming all-zero configuration")

    def _sleep(self, seconds):
        # Spin while sleeping so subscriptions (joint_states) keep updating;
        # required for correct grasp-hold detection inside service callbacks.
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)

    # ---------------- IK helpers ----------------
    def _tool_pose(self, pos, rot):
        return pose_from_rot_trans(rot, pos)

    def _solve(self, target_t, link=None):
        link = link or self.tool_link
        seeds = [self._current_q] + self.seed_sets
        return self.model.ik_retry(target_t, seeds, link=link)

    def _solve_home(self):
        pos = self.get_parameter("home_camera_pos").value
        look = self.get_parameter("home_camera_look").value
        cam_target = pose_from_rot_trans(look_at_matrix(pos, look), pos)
        tool_target = cam_target @ self.model.camera_in_tool()
        seeds = [self._current_q] + self.seed_sets
        q = self.model.ik_retry(tool_target, seeds)
        if q is None:
            raise RuntimeError("home pose IK failed; check home_camera_pos")
        return q

    # ---------------- low-level motion ----------------
    def _move_to_joints(self, target_q, duration):
        if duration <= 0:
            return True
        waypoints = interpolate_joints(
            self._current_q, target_q, steps=self.traj_steps
        )
        traj = JointTrajectory()
        traj.joint_names = list(self.model.arm_joints)
        for i, q in enumerate(waypoints):
            point = JointTrajectoryPoint()
            point.positions = [float(v) for v in q]
            point.time_from_start = rclpy.duration.Duration(
                seconds=duration * i / (len(waypoints) - 1)
            ).to_msg()
            traj.points.append(point)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        for _ in range(6):
            tol = JointTolerance()
            tol.position = 0.02
            tol.velocity = 0.05
            tol.name = self.model.arm_joints[_]
            goal.goal_tolerance.append(tol)
        goal.goal_time_tolerance = rclpy.duration.Duration(seconds=0.5).to_msg()

        future = self._arm_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.done() or future.result() is None:
            return False
        goal_handle = future.result()
        if not goal_handle.accepted:
            return False
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self, result_future, timeout_sec=duration + 8.0
        )
        if not result_future.done() or result_future.result() is None:
            return False
        self._current_q = np.array(target_q, dtype=float)
        return result_future.result().result.error_code in (
            FollowJointTrajectory.Result.SUCCESSFUL,
            0,
        )

    def _set_gripper(self, positions, duration):
        msg = Float64MultiArray()
        msg.data = [float(v) for v in positions]
        self._gripper_pub.publish(msg)
        self._sleep(duration)

    def _gripper_width(self):
        try:
            q_sum = (
                self._joint_positions["finger_left"]
                + self._joint_positions["finger_right"]
            )
        except KeyError:
            return 0.0
        return 0.04 - q_sum

    def _step(self, step):
        kind = step["kind"]
        if kind == "move":
            target = self._tool_pose(step["pos"], step["rot"])
            q = self._solve(target)
            if q is None or not self._move_to_joints(q, step["dur"]):
                self.get_logger().warn(f"move failed to {step['pos']}")
                return False
            return True
        if kind == "gripper":
            self._set_gripper(step["pos"], step["dur"])
            return True
        if kind == "hold_check":
            self._sleep(step["dur"])
            return self._gripper_width() > 0.007
        if kind == "view":
            cam_target = pose_from_rot_trans(
                look_at_matrix(step["pos"], step["look"]), step["pos"]
            )
            tool_target = cam_target @ self.model.camera_in_tool()
            q = self._solve(tool_target)
            if q is None or not self._move_to_joints(q, step["dur"]):
                return False
            self._sleep(step["dwell"])
            req = Observe.Request()
            req.auxiliary_view = True
            future = self._observe_cli.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            return future.done() and future.result() is not None
        if kind == "home":
            return self._move_to_joints(self.home_q, step["dur"])
        raise ValueError(f"unknown plan step kind: {kind}")

    # ---------------- services ----------------
    def _home_cb(self, request, response):
        response.success = self._move_to_joints(self.home_q, 1.2)
        response.message = "home reached" if response.success else "home failed"
        return response

    def _grasp_cb(self, request, response):
        u, v = int(request.u), int(request.v)
        z_top = float(request.height)
        if z_top < self.min_grasp_height:
            response.success = False
            response.object_cleared = False
            response.gripper_width = 0.0
            return response
        x, y = grid_to_xy(self.grid, u, v)
        plan = grasp_plan(self.params, x, y, z_top)

        held = False
        abort = False
        for step in plan:
            if step["kind"] == "hold_check":
                held = self._step(step)
                if not held:
                    abort = True
                    break
            else:
                if not self._step(step):
                    abort = True
                    break

        if abort:
            self._set_gripper(self.params["gripper_open"], 0.3)
            if not self._move_to_joints(self.home_q, 1.2):
                self.get_logger().warn("recovery: home move failed")

        response.success = held
        response.object_cleared = held
        response.gripper_width = self._gripper_width()
        return response

    def _push_cb(self, request, response):
        u, v = int(request.u), int(request.v)
        direction = int(request.direction) % 8
        z_top = float(request.height)
        if z_top < self.min_grasp_height:
            response.success = False
            return response
        x, y = grid_to_xy(self.grid, u, v)
        theta = direction * math.pi / 4.0
        plan = push_plan(self.params, x, y, theta)

        ok = True
        for step in plan:
            if not self._step(step):
                ok = False
                break
        if not ok:
            if not self._move_to_joints(self.home_q, 1.2):
                self.get_logger().warn("recovery: home move failed")
        response.success = ok
        return response

    def _view_cb(self, request, response):
        view_id = int(request.view_id)
        views = self.params["views"]
        if not 0 <= view_id < len(views):
            response.success = False
            return response
        plan = view_plan(self.params, view_id)
        ok = True
        for step in plan:
            if not self._step(step):
                ok = False
                break
        if not ok:
            if not self._move_to_joints(self.home_q, 1.2):
                self.get_logger().warn("recovery: home move failed")
        response.success = ok
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ExecutionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
