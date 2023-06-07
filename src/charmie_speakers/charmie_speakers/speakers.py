#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

class SpeakerNode(Node):

    def __init__(self):
        super().__init__("Speaker")
        self.get_logger().info("Initialised CHARMIE Speaker Node")

        self.speaker_command_subscriber = self.create_subscription(String, "speaker_command", self.speaker_command_callback, 10)
        self.flag_speech_done_publisher = self.create_publisher(Bool, "flag_speech_done", 10)
        
    def speaker_command_callback(self, speech: String):
        pass


def main(args=None):
    rclpy.init(args=args)
    node = SpeakerNode()
    rclpy.spin(node)
    rclpy.shutdown()
