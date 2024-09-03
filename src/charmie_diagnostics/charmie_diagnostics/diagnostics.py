#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, Vector3
from example_interfaces.msg import Bool, Int16
from sensor_msgs.msg import Image
from charmie_interfaces.msg import RobotSpeech

class BSNode(Node):

    def __init__(self):
        super().__init__("Diagnostics")
        self.get_logger().info("Initialised CHARMIE Diagnostics Node")
        
        ### Middleware ###
        """ # Navigation 
        self.navigation_diagnostic_subscriber = self.create_subscription(Bool, "navigation_diagnostic", self.navigation_diagnostic, 10)

        # Obstacle
        self.obstacles_diagnostic_subscriber = self.create_subscription(Bool, "obstacles_diagnostic", self.obstacles_diagnostic, 10)

        # Odometry
        self.odometry_diagnostic_subscriber = self.create_subscription(Bool, "odometry_diagnostic", self.odometry_diagnostic, 10)

        # Yolo Pose
        self.yolo_pose_diagnostic_subscriber = self.create_subscription(Bool, "yolo_pose_diagnostic", self.yolo_pose_diagnostic, 10)

        # Yolo Object
        self.yolo_object_diagnostic_subscriber = self.create_subscription(Bool, "yolo_object_diagnostic", self.yolo_object_diagnostic, 10)

        # Localisation
        self.localisation_diagnostic_subscriber = self.create_subscription(Bool, "localisation_diagnostic", self.localisation_diagnostic, 10)"""



        ### Hardware ###
        # Neck Topics
        self.neck_diagnostic_subscriber = self.create_subscription(Bool, "neck_diagnostic", self.neck_diagnostic, 10)

        # PS4 Controller
        self.ps4_diagnostic_subscriber = self.create_subscription(Bool, "ps4_diagnostic", self.ps4_diagnostic, 10)

        # LIDAR Hokuyo
        self.lidar_base_diagnostic_subscriber = self.create_subscription(Bool, "lidar_base_diagnostic", self.lidar_base_diagnostic, 10)

        # Speaker
        self.speakers_diagnostic_subscriber = self.create_subscription(Bool, "speakers_diagnostic", self.speakers_diagnostic, 10)
        self.flag_speaker_subscriber = self.create_subscription(Bool, "flag_speech_done", self.get_speech_done_callback, 10)

        # Speaker offline
        self.speakers_offline_diagnostic_subscriber = self.create_subscription(Bool, "speakers_offline_diagnostic", self.speakers_offline_diagnostic, 10)

        # Intel Realsense
        #self.intel_rs_diagnostic_subscriber = self.create_subscription(Bool, "intel_rs_diagnostic", self.intel_rs_diagnostic, 10)
        ##### This diagnostic has to be different from the others, since we don't have access to the package of the camera ####
        self.color_image_subscriber = self.create_subscription(Image, "/color/image_raw", self.get_color_image_callback, 10)

        # Face Shining RGB
        self.face_diagnostic_subscriber = self.create_subscription(Bool, "face_diagnostic", self.face_diagnostic, 10)

        # Audio
        self.audio_diagnostic_subscriber = self.create_subscription(Bool, "audio_diagnostic", self.audio_diagnostic, 10)


        
        ### Low Level ###
        # Low Level Topics
        self.low_level_diagnostic_subscriber = self.create_subscription(Bool, "low_level_diagnostic", self.low_level_diagnostic, 10)
        self.rgb_mode_publisher = self.create_publisher(Int16, "rgb_mode", 10)
        
        
        # Timers
        self.time = 15.0
        self.timer = self.create_timer(self.time, self.timer_callback)       

        # Get Flags
        self.flag_speech_done = False
        

        # Get Variables
        self.speech_str = RobotSpeech()
        self.rgb = Int16()

        self.nodes_launched = {
            ### SO FAR, IT IS NEEDED TO COMMENT / UNCOMMENT THE DESIRED NODES TO BE LAUNCHED IN THE LAUNCH FILE, SO THAT THE FEEDBACK FROM
            ### THIS NODE IS THE CORRECT. FOR EXAMPLE, IF I WANT TO LAUNCH A MODULE WHERE I WANT TO USE THE SPEAKERS_OFFLINE INSTEAD OF
            ### THE REGULAR SPEAKERS NODE, I JUST HAVE TO COMMENT ONE AND UNCOMMENT THE OTHER.

            'neck': False,
            'low_level': False,
            #'ps4': False,
            'lidar_base': False,
            #'depth_camera': False,
            'speakers': False,
            #'speakers_offline': False,
            'audio': False,
            #'face': False

            #---------------------------

            #'obstacles': False,
            #'odometry': False,
            #'navigation_sdnl': False,
            #'yolo_pose': False,
            #'yolo_objects': False,
            #'localisation': False,
            # Add more nodes as needed...
        }

        #Estou a pensar aqui fazer um print com o que n foram iniciados


    def timer_callback(self):
        unlaunched_nodes = self.get_unlaunched_nodes()
        unlaunched_node_list_str = ", ".join(unlaunched_nodes)
        self.get_logger().info(f"Unlaunched nodes: {unlaunched_nodes}")

        launched_nodes = self.get_launched_nodes()
        launched_node_list_str = ", ".join(launched_nodes)
        self.get_logger().info(f"Launched nodes: {launched_nodes}")

        if unlaunched_nodes == []:
            self.speech_str.command = f"All desired nodes have been launched correctly. Those are: {launched_node_list_str}"
            self.speaker_publisher.publish(self.speech_str)
            #self.wait_for_end_of_speaking()
            self.rgb_ctr = 12
            self.rgb.data = self.rgb_ctr
            self.rgb_mode_publisher.publish(self.rgb)

        else:
            self.speech_str.command = f"The following nodes have not been launched correctly: {unlaunched_node_list_str}"
            self.speaker_publisher.publish(self.speech_str)
            #self.wait_for_end_of_speaking()
            self.rgb_ctr = 2
            self.rgb.data = self.rgb_ctr
            self.rgb_mode_publisher.publish(self.rgb)   
    
    
    def wait_for_end_of_speaking(self):
        while not self.flag_speech_done:
            pass
        self.flag_speech_done = False

    def get_speech_done_callback(self, state: Bool):
        print("Received Speech Flag:", state.data)
        self.flag_speech_done = True
        #self.start_audio()

    def get_unlaunched_nodes(self):
        unlaunched_nodes = [node for node, launched in self.nodes_launched.items() if not launched]
        return unlaunched_nodes
    
    def get_launched_nodes(self):
        launched_nodes = [node for node, launched in self.nodes_launched.items() if launched]
        return launched_nodes

    def get_color_image_callback(self, img: Image):
        self.nodes_launched['depth_camera'] = True

    def neck_diagnostic(self, state: Bool):
        self.nodes_launched['neck'] = state.data
        #self.get_logger().info("Initialised CHARMIE Neck Node")

    def low_level_diagnostic(self, state: Bool):
        #self.get_logger().info("Initialised CHARMIE Low Level Node")
        self.nodes_launched['low_level'] = state.data

    def ps4_diagnostic(self, state: Bool):
        #self.get_logger().info("Initialised CHARMIE ps4 Node")
        self.nodes_launched['ps4'] = state.data

    def lidar_base_diagnostic(self, state: Bool):
        #self.get_logger().info("Initialised CHARMIE Lidar Base Node")
        self.nodes_launched['lidar_base'] = state.data

    """ def intel_rs_diagnostic(self, state: Bool):
        #self.get_logger().info("Initialised CHARMIE Intel RS Node")
        self.nodes_launched['depth_camera'] = state """

    def speakers_diagnostic(self, state: Bool):
        #self.get_logger().info("Initialised CHARMIE Speaker Node")
        self.nodes_launched['speakers'] = state.data

    def speakers_offline_diagnostic(self, state: Bool):
        #self.get_logger().info("Initialised CHARMIE Speakers Offline Node")
        self.nodes_launched['speakers_offline'] = state.data

    def face_diagnostic(self, state: Bool):
        #self.get_logger().info("Initialised CHARMIE Face Node")
        self.nodes_launched['face'] = state.data

    def audio_diagnostic(self, state: Bool):
        #self.get_logger().info("Initialised CHARMIE Audio Node")
        print(state)
        self.nodes_launched['audio'] = state.data

    """ def navigation_diagnostic(self, state: Bool):
        #self.get_logger().info("Initialised CHARMIE Navigation Node")
        self.nodes_launched['navigation_sdnl'] = state.data

    def obstacles_diagnostic(self, state: Bool):
        #self.get_logger().info("Initialised CHARMIE Obstacles Node")
        self.nodes_launched['obstacles'] = state.data

    def odometry_diagnostic(self, state: Bool):
        #self.get_logger().info("Initialised CHARMIE Odometry Node")
        self.nodes_launched['odometry'] = state.data

    def yolo_pose_diagnostic(self, state: Bool):
        #self.get_logger().info("Initialised CHARMIE Yolo Pose Node")
        self.nodes_launched['yolo_pose'] = state.data

    def yolo_object_diagnostic(self, state: Bool):
        #self.get_logger().info("Initialised CHARMIE Yolo Objects Node")
        self.nodes_launched['yolo_objects'] = state.data

    def localisation_diagnostic(self, state: Bool):
        #self.get_logger().info("Initialised CHARMIE Localisation Node")
        self.nodes_launched['localisation'] = state.data"""
   


def main(args=None):
    rclpy.init(args=args)
    node = BSNode()
    rclpy.spin(node)
    rclpy.shutdown()
