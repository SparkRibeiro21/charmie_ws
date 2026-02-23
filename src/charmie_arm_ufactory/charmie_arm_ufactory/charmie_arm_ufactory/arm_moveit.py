import rclpy
import math
from rclpy.node import Node
from rclpy.action import ActionClient

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from sensor_msgs.msg import JointState

from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs

from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest, RobotState
from geometry_msgs.msg import PoseStamped

class ArmMoveIt(Node):

    PLANNING_GROUP = "xarm6"
    BASE_FRAME     = "xarm_link_base"
    EE_LINK        = "xarm_link6"

    def __init__(self):
        super().__init__("arm_moveit")

        self.joint_names = [
            "xarm_joint1",
            "xarm_joint2",
            "xarm_joint3",
            "xarm_joint4",
            "xarm_joint5",
            "xarm_joint6",
        ]

        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/xarm6_controller/follow_joint_trajectory",
        )

        self.get_logger().info("Waiting for trajectory action server...")
        self._action_client.wait_for_server()
        self.get_logger().info("Trajectory action server ready.")

        self._current_joints: dict[str, float] = {}
        self.create_subscription(
            JointState,
            "/joint_states",
            self._joint_state_callback,
            10
        )

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._ik_client = self.create_client(GetPositionIK, "/compute_ik")
        self.get_logger().info("Waiting for IK service...")
        self._ik_client.wait_for_service()
        self.get_logger().info("IK service ready.")

    def _joint_state_callback(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            self._current_joints[name] = pos


    def set_move_tool(self, dx: float, dy: float, dz: float, duration_sec=3.0):
        """
        Move the tool by a relative offset in the base frame.

        dx, dy, dz: desired tool movement in meters
        """

        # Get current end-effector pose
        try:
            tf = self._tf_buffer.lookup_transform(
                self.BASE_FRAME,
                self.EE_LINK,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
        except Exception as e:
            self.get_logger().error(f"TF lookup failed: {e}")
            return
        
        t = tf.transform.translation
        r = tf.transform.rotation

        self.get_logger().info(
            f"Current EE pose: pos=({t.x:f}, {t.y:f}, {t.z:f}), "
            f"rot=({r.x:f}, {r.y:f}, {r.z:f}, {r.w:f})"
        )

        target_pose = PoseStamped()
        target_pose.header.frame_id = self.BASE_FRAME
        target_pose.pose.position.x = t.x + dx
        target_pose.pose.position.y = t.y + dy
        target_pose.pose.position.z = t.z + dz
        target_pose.pose.orientation = r

        self.get_logger().info(
            f"Target EE pose: pos=({target_pose.pose.position.x:f}, "
            f"{target_pose.pose.position.y:f}, {target_pose.pose.position.z:f}) "
        )

        ik_request = PositionIKRequest()
        ik_request.group_name = self.PLANNING_GROUP
        ik_request.ik_link_name = self.EE_LINK
        ik_request.pose_stamped = target_pose

        robot_state = RobotState()
        robot_state.joint_state.name = self.joint_names
        robot_state.joint_state.position = [
            self._current_joints.get(name, 0.0) for name in self.joint_names
        ]

        ik_request.robot_state = robot_state

        req = GetPositionIK.Request()
        req.ik_request = ik_request

        future = self._ik_client.call_async(req)
        future.add_done_callback(lambda f: self._ik_response_callback(f, duration_sec))
                

    # ------------------------------------------------------------------
    # PUBLIC API — equivalent to your old set_joint_values_
    # ------------------------------------------------------------------
    def send_joint_goal(self, positions_rad, duration_sec=3.0):
        """
        Send a single-point joint trajectory.

        positions_rad: list[float] in radians
        duration_sec: motion time
        """

        if len(positions_rad) != 6:
            self.get_logger().error("Expected 6 joint values.")
            return

        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = positions_rad
        point.time_from_start.sec = int(duration_sec)
        point.time_from_start.nanosec = int((duration_sec % 1) * 1e9)

        traj.points.append(point)

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = traj

        self.get_logger().info(f"Sending trajectory: {positions_rad}")

        future = self._action_client.send_goal_async(goal_msg)
        future.add_done_callback(self._goal_response_callback)

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def _goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Trajectory goal rejected.")
            return

        self.get_logger().info("Trajectory goal accepted.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Trajectory finished with error_code={result.error_code}")

    def _ik_response_callback(self, future, duration_sec: float):
            try:
                response = future.result()
            except Exception as e:
                self.get_logger().error(f"IK service call failed: {e}")
                return

            if response.error_code.val != 1:
                self.get_logger().error(f"IK solution not found: error_code={response.error_code.val}")
                return

            joint_positions = response.solution.joint_state
            joint_map = dict(zip(joint_positions.name, joint_positions.position))
            positions = [joint_map[n] for n in self.joint_names]

            self.get_logger().info(f"IK solution found: {positions}")   
            self.send_joint_goal(positions, duration_sec=duration_sec)


# ----------------------------------------------------------------------
# Standalone test
# ----------------------------------------------------------------------
def main(args=None):

    rclpy.init(args=args)
    node = ArmMoveIt()

    # Example:
    target = [-3.3672, 1.449, -1.1339, -0.017, 1.309, 4.712]

    node.send_joint_goal(target, duration_sec=4.0)


    node.create_timer(2.0, lambda: node.set_move_tool(dx=0.1, dy=0.0, dz=0.0, duration_sec=4.0))

    rclpy.spin(node)


if __name__ == "__main__":
    main()