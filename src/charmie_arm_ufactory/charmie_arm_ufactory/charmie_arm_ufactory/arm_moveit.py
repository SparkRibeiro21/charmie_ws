#!/usr/bin/env python3

import threading
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.time import Time

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest, RobotState
from geometry_msgs.msg import PoseStamped

from charmie_interfaces.srv import SetSimpleMoveTool


class ArmMoveIt(Node):
    PLANNING_GROUP = "xarm6"
    BASE_FRAME = "xarm_link_base"
    EE_LINK = "xarm_link6"

    JOINT_NAMES = [
        "xarm_joint1",
        "xarm_joint2",
        "xarm_joint3",
        "xarm_joint4",
        "xarm_joint5",
        "xarm_joint6",
    ]

    def __init__(self):
        super().__init__("arm_moveit")

        self._current_joints: dict[str, float] = {}

        # ReentrantCallbackGroup allows service callbacks to make
        # blocking calls on other clients/actions without deadlocking.
        self._cb_group = ReentrantCallbackGroup()

        # --- Action client ---
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/xarm6_controller/follow_joint_trajectory",
            callback_group=self._cb_group,
        )
        self.get_logger().info("Waiting for trajectory action server...")
        self._action_client.wait_for_server()
        self.get_logger().info("Trajectory action server ready.")

        # --- Joint state subscription ---
        self.create_subscription(
            JointState, "/joint_states", self._joint_state_cb, 10,
            callback_group=self._cb_group,
        )

        # --- TF ---
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._wait_for_tf()

        # --- IK service client ---
        self._ik_client = self.create_client(
            GetPositionIK, "/compute_ik",
            callback_group=self._cb_group,
        )
        self.get_logger().info("Waiting for IK service...")
        self._ik_client.wait_for_service()
        self.get_logger().info("IK service ready.")

        # --- SetMoveTool service server ---
        self.create_service(
            SetSimpleMoveTool, "set_simple_move_tool", self._handle_set_move_tool,
            callback_group=self._cb_group,
        )
        self.get_logger().info("SetMoveTool service ready.")

        # --- Block until we have real joint state data ---
        if not self._wait_for_joint_states():
            raise RuntimeError("Could not get joint states — aborting.")

    # ------------------------------------------------------------------ #
    #  Helper: block a service-handler thread on an async future          #
    #  WITHOUT calling spin — the MultiThreadedExecutor handles spinning. #
    # ------------------------------------------------------------------ #

    def _await_future(self, future):
        """
        Block the calling thread until `future` is resolved, then return
        its result.  Uses a threading.Event so the GIL is released and
        the executor's other threads keep processing callbacks normally.

        Never call rclpy.spin_until_future_complete() inside a callback
        that is already managed by a MultiThreadedExecutor — doing so
        creates a second spin loop that corrupts the executor's internal
        wait-set state and breaks subsequent calls.
        """
        event = threading.Event()
        future.add_done_callback(lambda _: event.set())
        event.wait()
        return future.result()

    # ------------------------------------------------------------------ #
    #  Helper: rotate a vector by a quaternion (tool → base frame)        #
    # ------------------------------------------------------------------ #

    @staticmethod
    def _rotate_vector_by_quat(v, q):
        """
        Rotate vector v by quaternion q = [x, y, z, w].
        Uses the efficient Rodrigues-like formula:
            t = 2 * (q_vec x v)
            v' = v + q_w * t + (q_vec x t)
        Pure numpy, no external quaternion libraries needed.
        """
        qx, qy, qz, qw = q
        q_vec = np.array([qx, qy, qz])
        t = 2.0 * np.cross(q_vec, v)
        return v + qw * t + np.cross(q_vec, t)

    # ------------------------------------------------------------------ #
    #  Subscriptions / wait helpers                                        #
    # ------------------------------------------------------------------ #

    def _joint_state_cb(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            self._current_joints[name] = pos

    def _wait_for_tf(self):
        self.get_logger().info("Waiting for TF...")
        while rclpy.ok():
            if self._tf_buffer.can_transform(
                self.BASE_FRAME, self.EE_LINK, Time(),
                timeout=Duration(seconds=0.5),
            ):
                break
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info("TF ready.")

    def _wait_for_joint_states(self, timeout_sec: float = 5.0) -> bool:
        self.get_logger().info("Waiting for joint states...")
        start = self.get_clock().now()
        while rclpy.ok():
            if all(n in self._current_joints for n in self.JOINT_NAMES):
                self.get_logger().info("Joint states ready.")
                return True
            elapsed = (self.get_clock().now() - start).nanoseconds / 1e9
            if elapsed > timeout_sec:
                self.get_logger().error(
                    f"Timed out waiting for joint states after {timeout_sec}s. "
                    f"Received: {list(self._current_joints.keys())}"
                )
                return False
            rclpy.spin_once(self, timeout_sec=0.1)
        return False

    def _joints_are_ready(self) -> bool:
        missing = [n for n in self.JOINT_NAMES if n not in self._current_joints]
        if missing:
            self.get_logger().error(f"Joint states incomplete — missing: {missing}")
            return False
        return True

    # ------------------------------------------------------------------ #
    #  Service handler — blocking, safe inside ReentrantCallbackGroup      #
    # ------------------------------------------------------------------ #

    def _handle_set_move_tool(self, request, response):
        """
        Fully blocking service handler.
        Safe because MultiThreadedExecutor + ReentrantCallbackGroup
        allows other callbacks to run on other threads while this blocks.

        The dx/dy/dz in the request are interpreted in the EE tool frame.
        They are rotated into the base frame before being passed to IK.
        """
        if not self._joints_are_ready():
            response.success = False
            response.message = "Joint states not ready."
            return response

        if not self._tf_buffer.can_transform(
            self.BASE_FRAME, self.EE_LINK, Time(), timeout=Duration(seconds=1.0)
        ):
            response.success = False
            response.message = "TF transform not available."
            return response

        # --- Look up current EE pose ---
        tf = self._tf_buffer.lookup_transform(self.BASE_FRAME, self.EE_LINK, Time())
        t = tf.transform.translation
        r = tf.transform.rotation

        self.get_logger().info(
            f"Current EE pose: pos=({t.x:.4f}, {t.y:.4f}, {t.z:.4f})  "
            f"rot=({r.x:.4f}, {r.y:.4f}, {r.z:.4f}, {r.w:.4f})"
        )

        # --- Rotate delta from tool frame into base frame ---
        delta_tool = np.array([request.dx, request.dy, request.dz])
        delta_base = self._rotate_vector_by_quat(delta_tool, [r.x, r.y, r.z, r.w])

        target = PoseStamped()
        target.header.frame_id = self.BASE_FRAME
        target.pose.position.x = t.x + delta_base[0]
        target.pose.position.y = t.y + delta_base[1]
        target.pose.position.z = t.z + delta_base[2]
        target.pose.orientation = r

        self.get_logger().info(
            f"Target EE pose: pos=({target.pose.position.x:.4f}, "
            f"{target.pose.position.y:.4f}, {target.pose.position.z:.4f})"
        )

        # --- Call IK service (blocking via threading.Event) ---
        robot_state = RobotState()
        robot_state.joint_state.name = self.JOINT_NAMES
        robot_state.joint_state.position = [
            self._current_joints[n] for n in self.JOINT_NAMES
        ]

        ik_req = PositionIKRequest()
        ik_req.group_name = self.PLANNING_GROUP
        ik_req.ik_link_name = self.EE_LINK
        ik_req.pose_stamped = target
        ik_req.robot_state = robot_state

        req = GetPositionIK.Request()
        req.ik_request = ik_req

        try:
            ik_response = self._await_future(self._ik_client.call_async(req))
        except Exception as e:
            response.success = False
            response.message = f"IK service call failed: {e}"
            return response

        if ik_response.error_code.val != 1:
            response.success = False
            response.message = f"IK failed: error_code={ik_response.error_code.val}"
            return response

        js = ik_response.solution.joint_state
        joint_map = dict(zip(js.name, js.position))
        positions = [joint_map[n] for n in self.JOINT_NAMES]
        self.get_logger().info(f"IK solution: {[f'{p:.4f}' for p in positions]}")

        # --- Send trajectory goal (blocking via threading.Event) ---
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = int(request.duration_sec)
        point.time_from_start.nanosec = int((request.duration_sec % 1) * 1e9)

        traj = JointTrajectory()
        traj.joint_names = self.JOINT_NAMES
        traj.points.append(point)

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = traj

        try:
            goal_handle = self._await_future(
                self._action_client.send_goal_async(goal_msg)
            )
        except Exception as e:
            response.success = False
            response.message = f"Send goal failed: {e}"
            return response

        if not goal_handle.accepted:
            response.success = False
            response.message = "Trajectory goal rejected."
            return response

        self.get_logger().info("Trajectory goal accepted, waiting for result...")

        try:
            result_response = self._await_future(goal_handle.get_result_async())
        except Exception as e:
            response.success = False
            response.message = f"Get result failed: {e}"
            return response

        result = result_response.result
        if result.error_code == 0:
            self.get_logger().info("Motion complete.")
            response.success = True
            response.message = "Motion complete."
        else:
            self.get_logger().error(f"Trajectory failed: error_code={result.error_code}")
            response.success = False
            response.message = f"Trajectory failed: error_code={result.error_code}"

        return response


# ---------------------------------------------------------------------- #
#  Entry point — MultiThreadedExecutor is required                        #
# ---------------------------------------------------------------------- #

def main(args=None):
    rclpy.init(args=args)
    node = ArmMoveIt()

    # MultiThreadedExecutor lets the service callback block on futures
    # while other callbacks (joint states, TF, etc.) keep running
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()