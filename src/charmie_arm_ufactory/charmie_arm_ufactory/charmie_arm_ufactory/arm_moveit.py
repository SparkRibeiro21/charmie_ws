import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class ArmMoveIt(Node):

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


# ----------------------------------------------------------------------
# Standalone test
# ----------------------------------------------------------------------
def main(args=None):

    rclpy.init(args=args)
    node = ArmMoveIt()

    # Example: small safe pose
    import math
    target = [-3.3672, 1.449, -1.1339, -0.017, 1.309, 4.712]

    node.send_joint_goal(target, duration_sec=4.0)

    rclpy.spin(node)


if __name__ == "main":
    main()