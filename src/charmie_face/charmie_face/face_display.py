#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from charmie_interfaces.srv import SetFace, SetTextFace
from sensor_msgs.msg import Image as Image_ ### ADD TO CHANGE IMAGE TO IMAGE_ because of: from PIL import Image
from realsense2_camera_msgs.msg import RGBD

from cv_bridge import CvBridge, CvBridgeError
import subprocess
import time
import os
from pathlib import Path
import pygame
import threading
from screeninfo import get_monitors
from PIL import Image
import cv2
import numpy as np


DEBUG_WITHOUT_DISPLAY = True

# ROS2 Face Node
class FaceNode(Node):
    def __init__(self):
        super().__init__("Face")
        self.get_logger().info("Initialised CHARMIE FACE Node")

        ### ROS2 Parameters ###
        # when declaring a ros2 parameter the second argument of the function is the default value 
        self.declare_parameter("show_speech", True) 
        self.declare_parameter("initial_face", "charmie_face") 
        # self.declare_parameter("initial_face", "place_bowl_in_tray") 
        # self.declare_parameter("initial_face", "charmie_face_old_tablet") 
        
        self.home = str(Path.home())
        midpath_faces = "/charmie_ws/src/charmie_face/charmie_face/"
        self.media_faces_path = self.home + midpath_faces + "list_of_media_faces/"
        self.temp_faces_path = self.home + midpath_faces + "list_of_temp_faces/"

        # Intel Cameras (Head and Hand/Gripper)
        self.rgbd_head_subscriber = self.create_subscription(RGBD, "/CHARMIE/D455_head/rgbd", self.get_rgbd_head_callback, 10)
        self.rgbd_hand_subscriber = self.create_subscription(RGBD, "/CHARMIE/D405_hand/rgbd", self.get_rgbd_hand_callback, 10)
        # Orbbec Camera (Base)
        self.color_image_base_subscriber = self.create_subscription(Image_, "/camera/color/image_raw", self.get_color_image_base_callback, 10)
        self.aligned_depth_image_base_subscriber = self.create_subscription(Image_, "/camera/depth/image_raw", self.get_depth_base_image_callback, 10)
        
        ### Services (Server) ###   
        self.server_face_command = self.create_service(SetFace, "face_command", self.callback_face_command) 
        self.speech_to_face_command = self.create_service(SetTextFace, "display_speech_face", self.callback_speech_to_face)
        self.get_logger().info("Face Servers have been started")

        # whether or not it is intended to show the speech strings on the face while the robot talks
        self.SHOW_SPEECH = self.get_parameter("show_speech").value
        # which face should be displayed after initialising the face node (string) 
        self.INITIAL_FACE = self.get_parameter("initial_face").value
        
        self.get_logger().info("Initial Face Received is: %s" %self.INITIAL_FACE)

        # the time after every speaked sentence, that the face remains the speech after finished the speakers (float) 
        self.AFTER_SPEECH_TIMER_SHORT = 0.25
        self.AFTER_SPEECH_TIMER_LONG = 1.0

        self.new_face_received = False
        self.new_face_received_name = ""

        self.new_text_received = False
        self.new_text_received_name = ""
        self.new_text_received_delay = self.AFTER_SPEECH_TIMER_SHORT

        self.cams_flag = False
        self.selected_camera_stream = "head"

        self.HEAD_CAM_WIDTH = 848
        self.BASE_CAM_WIDTH = 640
        self.HEAD_CAM_HEIGHT = 480
        self.head_rgb =   np.zeros((self.HEAD_CAM_HEIGHT, self.HEAD_CAM_WIDTH, 3), np.uint8)
        self.hand_rgb =   np.zeros((self.HEAD_CAM_HEIGHT, self.HEAD_CAM_WIDTH, 3), np.uint8)
        self.base_rgb =   np.zeros((self.HEAD_CAM_HEIGHT, self.BASE_CAM_WIDTH, 3), np.uint8)
        self.head_depth = np.zeros((self.HEAD_CAM_HEIGHT, self.HEAD_CAM_WIDTH, 3), np.uint8)
        self.hand_depth = np.zeros((self.HEAD_CAM_HEIGHT, self.HEAD_CAM_WIDTH, 3), np.uint8)
        self.base_depth = np.zeros((self.HEAD_CAM_HEIGHT, self.BASE_CAM_WIDTH, 3), np.uint8)
        self.new_head_rgb = False
        self.new_hand_rgb = False
        self.new_base_rgb = False
        self.new_head_depth = False
        self.new_hand_depth = False
        self.new_base_depth = False

        self.br = CvBridge()
        
        # sends initial face
        self.image_to_face(self.INITIAL_FACE)
        # easier debug when testing custom faces 
        # self.image_to_face("charmie_face_green")
    
    # Callback for all face commands received
    def callback_face_command(self, request, response):
        
        # Type of service received: 
        # string command          # type of face that is commonly used and is always the same, already in face (i.e. hearing face and standard blinking eyes face)
        # string custom           # type of face that is custom, not previously in face (i.e. show detected person or object in the moment)
        # string camera           # select which camera must be shown in face (can be rgb or depth)
        # bool show_detections    # select if in addition to show the camera on the face, shows the detections being used with that camera

        self.cams_flag = False
        if request.command != "":
            response.success, response.message = self.image_to_face(command=request.command)
        elif request.custom != "":
            response.success, response.message = self.custom_image_to_face(command=request.custom)
        elif request.camera != "":
            self.cams_flag = True
            self.selected_camera_stream = request.camera.lower()
        else:
            response.success = False
            response.message = "No standard or custom face received."

        print("Face Request:", request.command, request.custom, request.camera, request.show_detections)

        return response

    # Receive speech strings to show in face
    def callback_speech_to_face(self, request, response):
        
        # Type of service received: 
        # string data     # informational, e.g. for error messages.
        # bool long_pause # whether after showing in face, a long or a short pause should be added, for user easier reading
        # ---
        # bool success   # indicate successful run of triggered service
        # string message # informational, e.g. for error messages.

        print("Received (text):", request.data)

        if self.SHOW_SPEECH:
            if request.data != "":
                self.new_text_received = True
                self.new_text_received_name = request.data
                self.get_logger().info("FACE received (text) - %s" %request.data)
                # print("Received Speech String:", command.data)
            else:
                self.new_text_received = True
                self.new_text_received_name = request.data
                if request.long_pause:
                    self.new_text_received_delay = self.AFTER_SPEECH_TIMER_LONG + self.AFTER_SPEECH_TIMER_SHORT
                else:
                    self.new_text_received_delay = self.AFTER_SPEECH_TIMER_SHORT
        
        response.success = True
        response.message = "Received and displayed text on face."

        return response
                    
    # Receive image or video files name to show in face
    def image_to_face(self, command):
        # self.get_logger().info("init image to face")
        # since the extension is not known, a system to check all filenames disregarding the extension had to be created
        file_exists = False
        files = os.listdir(self.media_faces_path)
        correct_extension = ""
        # self.get_logger().info("pre  GET FILES")
        for file in files:
            file_name, file_extension = os.path.splitext(file)
            if file_name == command:
                correct_extension = file_extension
                file_exists = True
        
        if file_exists:
            self.get_logger().info("FACE received (standard) - %s" %command)
            # self.face.save_text_file("media/" + command)
            self.new_face_received = True
            self.new_face_received_name = self.media_faces_path + command + correct_extension
            return True, "Face received (standard) sucessfully displayed"
        
        else:
            self.get_logger().error("FACE received (standard) does not exist! - %s" %command)
            return False, "FACE received (standard) does not exist."
        
    # Receive custom image name to send to tablet and show in face
    def custom_image_to_face(self, command):

        # checks whether file exists, maybe there was some typo 
        file_exists = os.path.exists(self.temp_faces_path + command + ".jpg")
        
        if file_exists:
            self.get_logger().info("FACE received (standard) - %s" %command)
            # self.face.save_text_file("media/" + command)
            self.new_face_received = True
            self.new_face_received_name = self.temp_faces_path + command + ".jpg"
            return True, "Face received (standard) sucessfully displayed"

        else:
            self.get_logger().error("FACE received (custom) does not exist! - %s" %command)
            return False, "FACE received (custom) does not exist."
    
    # CAMERAS 
    def get_rgbd_head_callback(self, rgbd: RGBD):
        self.head_rgb = cv2.cvtColor(self.br.imgmsg_to_cv2(rgbd.rgb, "bgr8"), cv2.COLOR_BGR2RGB)
        self.head_depth = self.get_cv2_cvtColor_from_depth_image(rgbd.depth, "head")
        self.new_head_rgb = True
        self.new_head_depth = True
        # print("HEAD:", rgbd.rgb_camera_info.height, rgbd.rgb_camera_info.width, rgbd.depth_camera_info.height, rgbd.depth_camera_info.width)

    def get_rgbd_hand_callback(self, rgbd: RGBD):
        self.hand_rgb = cv2.cvtColor(self.br.imgmsg_to_cv2(rgbd.rgb, "bgr8"), cv2.COLOR_BGR2RGB)
        self.hand_depth = self.get_cv2_cvtColor_from_depth_image(rgbd.depth, "hand")
        self.new_hand_rgb = True
        self.new_hand_depth = True
        # print("HAND:", rgbd.rgb_camera_info.height, rgbd.rgb_camera_info.width, rgbd.depth_camera_info.height, rgbd.depth_camera_info.width)

    def get_color_image_base_callback(self, img: Image):
        self.base_rgb = cv2.cvtColor(self.br.imgmsg_to_cv2(img, "bgr8"), cv2.COLOR_BGR2RGB)
        self.new_base_rgb = True

    def get_depth_base_image_callback(self, img: Image):
        self.base_depth = self.get_cv2_cvtColor_from_depth_image(img, "base")
        self.new_base_depth = True

    def get_cv2_cvtColor_from_depth_image(self, cam, name):

        opencv_depth_image = self.br.imgmsg_to_cv2(cam, "passthrough")

        min_val = 0
        if name == "head":
            max_val = 6000
        elif name == "hand":
            max_val = 1000
        elif name == "base":
            max_val = 6000
        
        depth_normalized = (opencv_depth_image - min_val) / (max_val - min_val)
        depth_normalized = np.clip(depth_normalized, 0, 1)
        
        # Convert the normalized depth image to an 8-bit image (0-255)
        depth_8bit = (depth_normalized * 255).astype(np.uint8)

        # Apply a colormap to the 8-bit depth image
        opencv_depth_image = cv2.applyColorMap(depth_8bit, cv2.COLORMAP_JET)
        
        # Convert the image to RGB (OpenCV loads as BGR by default)
        return cv2.cvtColor(opencv_depth_image, cv2.COLOR_BGR2RGB)

def main(args=None):
    rclpy.init(args=args)
    node = FaceNode()
    th_face_display = threading.Thread(target=ThreadMainFace, args=(node,), daemon=True)
    th_face_display.start()
    rclpy.spin(node)
    rclpy.shutdown()

def ThreadMainFace(node: FaceNode):
    main = FaceMain(node)
    main.main()


class FaceMain():

    def __init__(self, node: FaceNode):
        # create a ROS2 node instance
        self.node = node

        if not DEBUG_WITHOUT_DISPLAY:
            self.device_id = self.get_touchscreen_id("Waveshare")  # Or any keyword matching the device
            self.display_name = "DP-1-2"
            self.map_touchscreen_to_correct_display(device_id=self.device_id, display_name=self.display_name)
            self.resolution = self.get_display_resolution()
            self.SCREEN = self.initiliase_pygame_screen(screen=1)
        else:
            self.resolution = [1280, 800]
            self.SCREEN = self.initiliase_pygame_screen(screen=1)
            
        self.running = True
        self.gif_flag = False
        self.gif_frames = []
        self.frame_index = 0
        self.clock = pygame.time.Clock()
        self.previous_image_extension = ""

        self.font = pygame.font.SysFont("Comic Sans MS", 150)  # (font name, font size) – you can change!
        self.font_color = (0, 0, 0)  # White color for the text
        
        self.xx_shift = 0.0
        self.yy_shift = 0.0

    def get_touchscreen_id(self, name_contains="touch"):

        # xinput list
        # xrandr
        result = subprocess.run(["xinput", "list"], capture_output=True, text=True)
        for line in result.stdout.splitlines():
            if name_contains.lower() in line.lower():
                parts = line.strip().split('\t')
                for part in parts:
                    if part.startswith("id="):
                        return part.split('=')[1]
        return None

    def map_touchscreen_to_correct_display(self, device_id, display_name):
        # this maps the input of the touchscrren to the correct display 
        if device_id:
            try:
                subprocess.run(["xinput", "map-to-output", device_id, display_name], check=True)
                print(f"Touchscreen {device_id} successfully mapped to {display_name}")
            except subprocess.CalledProcessError as e:
                print(f"Failed to map: {e}")
        else:
            print("Touchscreen not found.")

        print(device_id)

    def get_display_resolution(self):
        
        resolution = [0, 0]
        # Tihs loop goes through all monitors and checks the resolution
        for m in get_monitors():
            # print(m)
            if(m.is_primary == False):
                resolution[0] = m.width
                resolution[1] = m.height
        
        return resolution
    
    def initiliase_pygame_screen(self, screen=0):

        pygame.init()
        flags = pygame.DOUBLEBUF | pygame.NOFRAME
        SCREEN = pygame.display.set_mode(tuple(self.resolution), flags, 8, display=screen, vsync=1)
        # Aui o display = 0, é que define para por no ecra principal, se puseres 1 mete no secundario e assim sucessivamente
        # As flags de DOUBLEBUF e NOFRAME é so para correr um bocadinho mais rapido em fullscreen, nao faz grande diferença mas prontos
        pygame.display.set_caption("Main Window")

        logo_midpath = "/charmie_ws/src/configuration_files/docs/logos/"

        icon = pygame.image.load(self.node.home+logo_midpath+"charmie_face.png")
        pygame.display.set_icon(icon)

        return SCREEN
    
    def dynamic_image_resize(self, image):
    
        # get image and screen size
        image_width, image_height = image.get_size()
        screen_width, screen_height = self.SCREEN.get_size()

        # rates from screen size to image size
        x_rate = image_width/screen_width
        y_rate = image_height/screen_height

        # adjust width and height depending on rates
        if x_rate < y_rate:
            width  = int(image_width/y_rate)
            height = int(image_height/y_rate)
        else:
            width  = int(image_width/x_rate)
            height = int(image_height/x_rate)

       # Compute top-left position to center the image
        self.xx_shift = (screen_width  - width)  // 2
        self.yy_shift = (screen_height - height) // 2

        # print("-")
        # print(image_width, image_height)
        # print(screen_width, screen_height)
        # print(x_rate, y_rate) 
        # print(self.xx_shift, self.yy_shift)
        # print(width, height)

        return (width, height)

    def update_received_face(self):
                
        self.node.new_face_received = False
        self.gif_flag = False
        print(self.node.new_face_received_name)
        print("New Face Received in Pygame")

        file_name, file_extension = os.path.splitext(self.node.new_face_received_name)

        print(file_extension)
        if file_extension == ".jpg" or file_extension == ".jpeg" or file_extension == ".png":
            self.gif_flag = False
            self.image = pygame.image.load(self.node.new_face_received_name)
            self.image = pygame.transform.scale(self.image, self.dynamic_image_resize(self.image))
        
        elif file_extension == ".gif":
            self.gif_flag = True
            gif = Image.open(self.node.new_face_received_name)
            self.gif_frames.clear()
            if self.previous_image_extension != ".gif":
                self.frame_index = 0 # I do not reset the frame_index! 
            
            # self.frame_index = 0 # I do not reset the frame_index when the last face is a gif! 
            # This way, when changing between faces (since all of them have the same number of frames),
            # if the face is making any movement, the new face will continue that movement (eyes or mouth moving)
            
            for frame in range(gif.n_frames):
                gif.seek(frame)
                frame = pygame.image.fromstring(gif.tobytes(), gif.size, gif.mode)
                frame = pygame.transform.scale(frame, self.dynamic_image_resize(frame))
                self.gif_frames.append(frame)
        else:
            pass

        self.previous_image_extension = file_extension
    
    def wrap_text(self, text, font, max_width):
        words = text.split(' ')
        lines = []
        current_line = ''

        for word in words:
            # Check the width if we add the next word
            test_line = current_line + ' ' + word if current_line else word
            if font.size(test_line)[0] <= max_width:
                current_line = test_line
            else:
                lines.append(current_line)
                current_line = word

        if current_line:
            lines.append(current_line)

        return lines
        
    def add_text_to_face(self):
        
        # Wrap the text into multiple lines
        lines = self.wrap_text(self.node.new_text_received_name, self.font, self.resolution[0] - 50)  # 50 pixels margin

        # First render all lines into surfaces
        rendered_lines = []
        max_width = 0
        total_height = 0

        for line in lines:
            surface = self.font.render(line, True, self.font_color)
            rect = surface.get_rect()
            rendered_lines.append((surface, rect))
            max_width = max(max_width, rect.width)
            total_height += rect.height

        # Calculate start position for vertical centering
        start_y = (self.resolution[1] - total_height) // 2

        # Draw all lines centered
        current_y = start_y
        for surface, rect in rendered_lines:
            rect.centerx = self.resolution[0] // 2
            rect.top = current_y
            self.SCREEN.blit(surface, rect)
            current_y += rect.height
                
    def main(self):
        
        while self.running:
            
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False
            
            if self.node.new_text_received:
                self.SCREEN.fill((216, 231, 240))  # clear screen (black background)
                if self.node.new_text_received_name != "":
                    self.add_text_to_face()
                    pygame.display.update()
                else: # == ""
                    start_time = time.time()
                    self.node.new_text_received = False
                    while time.time() - start_time < self.node.new_text_received_delay:
                        pass # stales the display thread, so that new faces may be received, however the text has higher priority
            
            elif self.node.cams_flag:

                # camera stream selection
                if self.node.selected_camera_stream == "head":
                    selected_video_stream = self.node.head_rgb
                elif self.node.selected_camera_stream == "hand":
                    selected_video_stream = self.node.hand_rgb
                elif self.node.selected_camera_stream == "base":
                    selected_video_stream = self.node.base_rgb
                elif self.node.selected_camera_stream == "head depth":
                    selected_video_stream = self.node.head_depth
                elif self.node.selected_camera_stream == "hand depth":
                    selected_video_stream = self.node.hand_depth
                elif self.node.selected_camera_stream == "base depth":
                    selected_video_stream = self.node.base_depth
                else:
                    self.node.cams_flag = False
                    self.node.image_to_face(self.node.INITIAL_FACE) # sets default face

                if self.node.cams_flag: # this way it skips showing camera stream if no valid camera was received, and changes to default face 

                    surface = pygame.surfarray.make_surface(np.transpose(selected_video_stream, (1, 0, 2)))  # Pygame expects (width, height, channels)
                    scaled_surface = pygame.transform.scale(surface, self.dynamic_image_resize(surface))
                    self.SCREEN.fill((0, 0, 0))  # cleans display to make sure if the new image does not use all pixels you can not see the pixels from last image on non-used pixels
                    self.SCREEN.blit(scaled_surface, (self.xx_shift, self.yy_shift))
                    pygame.display.update()
                    time.sleep(0.05)

            else:
                if self.node.new_face_received:
                    self.update_received_face()
                    self.SCREEN.fill((0, 0, 0))  # cleans display to make sure if the new image does not use all pixels you can not see the pixels from last image on non-used pixels

                if self.gif_flag:
                    if self.frame_index >= len(self.gif_frames): # safety for gifs with a higher amount of frames stop and a new one with less frames wants to be used 
                        self.frame_index = 1
                    self.SCREEN.blit(self.gif_frames[self.frame_index], (self.xx_shift, self.yy_shift))
                    pygame.display.update()
                    self.frame_index = (self.frame_index + 1) % len(self.gif_frames)
                    if self.frame_index == 0: # needs this line to avoid a blank frame everytime the gif resets
                        self.frame_index = 1
                    # print(self.frame_index)
                    self.clock.tick(30)
                elif self.previous_image_extension != "": # use self.previous_image_extension for initial case
                    # self.SCREEN.blit(self.image, (0, 0))
                    self.SCREEN.blit(self.image, (self.xx_shift, self.yy_shift))
                    pygame.display.update()

        pygame.quit()
