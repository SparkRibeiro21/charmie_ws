#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, String, Float32
from charmie_interfaces.msg import SpeechType, RobotSpeech

class PersonRec():
    def __init__(self):
        print("New Person Recognition Class Initialised")

    # your code here

class PersonRecognitionNode(Node):

    def __init__(self):
        super().__init__("PersonRecognition")
        self.get_logger().info("Initialised CHARMIE Person Recognition Node")

        self.speaker_publisher = self.create_publisher(RobotSpeech, "speech_command", 10)        
        self.flag_speaker_subscriber = self.create_subscription(Bool, "flag_speech_done", self.get_speech_done_callback, 10)
        
        self.robot = PersonRec()

    def get_speech_done_callback(self, state: Bool):
        print("Received Speech Flag:", state.data)
        self.get_logger().info("Received Speech Flag")

    
def main(args=None):
    rclpy.init(args=args)
    node = PersonRecognitionNode()
    rclpy.spin(node)
    rclpy.shutdown()