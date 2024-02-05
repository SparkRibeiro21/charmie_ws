#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, String, Float32
from charmie_interfaces.msg import SpeechType, RobotSpeech


class Face():
    def __init__(self):
        print("New Face Class Initialised")


class FaceNode(Node):
    def __init__(self):
        super().__init__("Face")
        self.get_logger().info("Initialised CHARMIE Face Node")

        self.speaker_publisher = self.create_publisher(RobotSpeech, "speech_command", 10)        
        self.flag_speaker_subscriber = self.create_subscription(Bool, "flag_speech_done", self.get_speech_done_callback, 10)
        
        self.face = Face()

    def get_speech_done_callback(self, state: Bool):
        print("Received Speech Flag:", state.data)
        self.get_logger().info("Received Speech Flag")


def main(args=None):
    rclpy.init(args=args)
    node = FaceNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
