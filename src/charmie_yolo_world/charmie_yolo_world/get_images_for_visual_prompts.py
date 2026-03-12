#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from realsense2_camera_msgs.msg import RGBD
from cv_bridge import CvBridge
import cv2 
import threading
import numpy as np
from pathlib import Path
from datetime import datetime

data_lock = threading.Lock()


class Cam_node(Node):
    def __init__(self):
        super().__init__("get_image_visual_prompt")
        self.get_logger().info("Initialised Get Images for Visual Prompts Node")

        # info regarding the paths for the recorded files intended to be played
        # by using self.home it automatically adjusts to all computers home file, which may differ since it depends on the username on the PC
        self.home = str(Path.home())
        midpath_visual_prompt_images = "charmie_ws/src/charmie_yolo_world/charmie_yolo_world/visual_prompt_images"
        self.complete_path_visual_prompts = self.home + "/" + midpath_visual_prompt_images + "/"

        ### Topics ###
        # Intel Realsense Subscribers (RGBD) Head and Hand Cameras
        self.rgbd_head_subscriber = self.create_subscription(RGBD, "/CHARMIE/D455_head/rgbd", self.get_rgbd_head_callback, 10)
        self.rgbd_hand_subscriber = self.create_subscription(RGBD, "/CHARMIE/D405_hand/rgbd", self.get_rgbd_hand_callback, 10)
        # Orbbec Camera (Base)
        self.color_image_base_subscriber = self.create_subscription(Image, "/camera/color/image_raw", self.get_color_image_base_callback, 10)
        # self.aligned_depth_image_base_subscriber = self.create_subscription(Image, "/camera/depth/image_raw", self.get_depth_base_image_callback, 10)

        ### Variables ###        
        self.br = CvBridge()

        self.CAM_IMAGE_WIDTH = 848
        self.CAM_BASE_IMAGE_WIDTH = 640
        self.CAM_IMAGE_HEIGHT = 480

        self.head_rgb_cv2_frame = np.zeros((self.CAM_IMAGE_HEIGHT, self.CAM_IMAGE_WIDTH,      3), np.uint8)
        self.hand_rgb_cv2_frame = np.zeros((self.CAM_IMAGE_HEIGHT, self.CAM_IMAGE_WIDTH,      3), np.uint8)
        self.base_rgb_cv2_frame = np.zeros((self.CAM_IMAGE_HEIGHT, self.CAM_BASE_IMAGE_WIDTH, 3), np.uint8)
        
        self.got_head = False
        self.got_hand = False
        self.got_base = False


    def get_rgbd_head_callback(self, rgbd: RGBD):
        with data_lock:
            self.head_rgb_cv2_frame = self.br.imgmsg_to_cv2(rgbd.rgb, "bgr8")
            self.got_head = True

    def get_rgbd_hand_callback(self, rgbd: RGBD):
        with data_lock:
            self.hand_rgb_cv2_frame = self.br.imgmsg_to_cv2(rgbd.rgb, "bgr8")
            self.got_hand = True

    def get_color_image_base_callback(self, img: Image):
        with data_lock:
            self.base_rgb_cv2_frame = self.br.imgmsg_to_cv2(img, "bgr8")
            self.got_base = True

    def save_latest(self, cam: str):
        with data_lock:
            if cam == "head":
                if not self.got_head:
                    self.get_logger().warn("No head frame received yet.")
                    return
                frame = self.head_rgb_cv2_frame.copy()
            elif cam == "hand":
                if not self.got_hand:
                    self.get_logger().warn("No hand frame received yet.")
                    return
                frame = self.hand_rgb_cv2_frame.copy()
            elif cam == "base":
                if not self.got_base:
                    self.get_logger().warn("No base frame received yet.")
                    return
                frame = self.base_rgb_cv2_frame.copy()
            else:
                self.get_logger().warn(f"Unknown cam '{cam}'. Use h/g/b or head/hand/base.")
                return

        ts = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        filename = f"{ts}_{cam}.jpg"
        out_path = str(Path(self.complete_path_visual_prompts) / filename)

        ok = cv2.imwrite(out_path, frame)
        if ok:
            self.get_logger().info(f"Saved {cam} image -> {out_path}")
        else:
            self.get_logger().error(f"Failed to save image -> {out_path}")


# main function that already creates the thread for the task state machine
def main(args=None):
    rclpy.init(args=args)
    node = Cam_node()
    th_main = threading.Thread(target=ThreadMainGetCamImages, args=(node,), daemon=True)
    th_main.start()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

def ThreadMainGetCamImages(node: Cam_node):
    main = GetCamImagesMain(node)
    main.main()


class GetCamImagesMain():

    def __init__(self, node: Cam_node):
        # create a node instance so all variables ros related can be acessed
        self.node = node
        
    # main state-machine function
    def main(self):
        print("Type: h=head, g=hand, b=base (press Enter). Type q to quit.")
        while rclpy.ok():
            try:
                s = input("> ").strip().lower()
            except (EOFError, KeyboardInterrupt):
                s = "q"

            if s in ("q", "quit", "exit"):
                self.node.get_logger().info("Quit requested.")
                rclpy.shutdown()
                return

            # accept single letters or words
            if s in ("h", "head"):
                self.node.save_latest("head")
            elif s in ("g", "hand", "gripper", "griper"): # prevents typos in gripper
                self.node.save_latest("hand")
            elif s in ("b", "base"):
                self.node.save_latest("base")
            else:
                print("Unknown. Use: h/g/b (or head/hand/base).")