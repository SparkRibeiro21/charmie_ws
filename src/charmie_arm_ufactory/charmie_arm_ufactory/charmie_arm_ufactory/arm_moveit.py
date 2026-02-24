import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.time import Time

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest, RobotState
from geometry_msgs.msg import PoseStamped


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

        # --- Action client ---
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/xarm6_controller/follow_joint_trajectory",
        )
        self.get_logger().info("Waiting for trajectory action server...")
        self._action_client.wait_for_server()
        self.get_logger().info("Trajectory action server ready.")

        # --- Joint state subscription ---
        self.create_subscription(JointState, "/joint_states", self._joint_state_cb, 10)

        # --- TF ---
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._wait_for_tf()

        # --- IK service client ---
        self._ik_client = self.create_client(GetPositionIK, "/compute_ik")
        self.get_logger().info("Waiting for IK service...")
        self._ik_client.wait_for_service()
        self.get_logger().info("IK service ready.")

        # --- Block until we have real joint state data ---
        if not self._wait_for_joint_states():
            raise RuntimeError("Could not get joint states — aborting.")

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
                self.BASE_FRAME,
                self.EE_LINK,
                Time(),
                timeout=Duration(seconds=0.5),
            ):
                break
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info("TF ready.")

    def _wait_for_joint_states(self, timeout_sec: float = 5.0) -> bool:
        """Block until all joints have been received at least once."""
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
        """Runtime guard — confirm all joints are still present."""
        missing = [n for n in self.JOINT_NAMES if n not in self._current_joints]
        if missing:
            self.get_logger().error(
                f"Joint states incomplete — missing: {missing}"
            )
            return False
        return True

    # ------------------------------------------------------------------ #
    #  Public API                                                          #
    # ------------------------------------------------------------------ #

    def send_joint_goal(self, positions_rad: list[float], duration_sec: float = 3.0):
        """Send a single-point joint trajectory goal (non-blocking)."""
        if len(positions_rad) != len(self.JOINT_NAMES):
            self.get_logger().error(
                f"Expected {len(self.JOINT_NAMES)} joint values, got {len(positions_rad)}."
            )
            return

        point = JointTrajectoryPoint()
        point.positions = list(positions_rad)
        point.time_from_start.sec = int(duration_sec)
        point.time_from_start.nanosec = int((duration_sec % 1) * 1e9)

        traj = JointTrajectory()
        traj.joint_names = self.JOINT_NAMES
        traj.points.append(point)

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = traj

        self.get_logger().info(f"Sending trajectory goal: {positions_rad}")
        future = self._action_client.send_goal_async(goal_msg)
        future.add_done_callback(self._on_goal_response)

    def set_move_tool(self, dx: float, dy: float, dz: float, duration_sec: float = 3.0):
        """Move the tool-center-point by (dx, dy, dz) in the base frame (non-blocking)."""
        # Guard 1: joint states must be populated with real data
        if not self._joints_are_ready():
            return

        # Guard 2: TF must be available
        if not self._tf_buffer.can_transform(
            self.BASE_FRAME, self.EE_LINK, Time(), timeout=Duration(seconds=1.0)
        ):
            self.get_logger().error("TF transform not available.")
            return

        tf = self._tf_buffer.lookup_transform(self.BASE_FRAME, self.EE_LINK, Time())
        t = tf.transform.translation
        r = tf.transform.rotation

        self.get_logger().info(
            f"Current EE pose: pos=({t.x:.4f}, {t.y:.4f}, {t.z:.4f})  "
            f"rot=({r.x:.4f}, {r.y:.4f}, {r.z:.4f}, {r.w:.4f})"
        )

        target = PoseStamped()
        target.header.frame_id = self.BASE_FRAME
        target.pose.position.x = t.x + dx
        target.pose.position.y = t.y + dy
        target.pose.position.z = t.z + dz
        target.pose.orientation = r

        self.get_logger().info(
            f"Target  EE pose: pos=({target.pose.position.x:.4f}, "
            f"{target.pose.position.y:.4f}, {target.pose.position.z:.4f})"
        )

        # Seed IK with current joint positions (guaranteed non-zero here)
        robot_state = RobotState()
        robot_state.joint_state.name = self.JOINT_NAMES
        robot_state.joint_state.position = [
            self._current_joints[n] for n in self.JOINT_NAMES  # no .get() fallback
        ]

        ik_req = PositionIKRequest()
        ik_req.group_name = self.PLANNING_GROUP
        ik_req.ik_link_name = self.EE_LINK
        ik_req.pose_stamped = target
        ik_req.robot_state = robot_state

        req = GetPositionIK.Request()
        req.ik_request = ik_req

        future = self._ik_client.call_async(req)
        future.add_done_callback(
            lambda f, d=duration_sec: self._on_ik_response(f, d)
        )

    # ------------------------------------------------------------------ #
    #  Private callbacks                                                   #
    # ------------------------------------------------------------------ #

    def _on_ik_response(self, future, duration_sec: float):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f"IK service call failed: {e}")
            return

        if response.error_code.val != 1:
            self.get_logger().error(f"IK failed: error_code={response.error_code.val}")
            return

        js = response.solution.joint_state
        joint_map = dict(zip(js.name, js.position))
        positions = [joint_map[n] for n in self.JOINT_NAMES]
        self.get_logger().info(f"IK solution: {[f'{p:.4f}' for p in positions]}")
        self.send_joint_goal(positions, duration_sec)

    def _on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Trajectory goal rejected.")
            return
        self.get_logger().info("Trajectory goal accepted.")
        goal_handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future):
        result = future.result().result
        if result.error_code == 0:
            self.get_logger().info("Motion complete.")
        else:
            self.get_logger().error(
                f"Trajectory failed: error_code={result.error_code}"
            )


# ---------------------------------------------------------------------- #
#  Entry point                                                            #
# ---------------------------------------------------------------------- #

def main(args=None):
    rclpy.init(args=args)
    node = ArmMoveIt()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()