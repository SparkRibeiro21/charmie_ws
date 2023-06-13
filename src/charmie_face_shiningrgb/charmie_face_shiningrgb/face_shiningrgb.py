#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class FaceNode(Node):

    def __init__(self):
        super().__init__("Face")
        self.get_logger().info("Initialised CHARMIE Face Node")

        self.speaker_command_subscriber = self.create_subscription(Bool, "face_command", self.face_command_callback, 10)
        
    def face_command_callback(self, command: Bool):
        print("Received Command:", command.data)

        
def main(args=None):
    rclpy.init(args=args)
    node = FaceNode()
    rclpy.spin(node)
    rclpy.shutdown()
