#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from charmie_interfaces.srv import SetFace, SetTextFace, SetFaceTouchscreenMenu, GetFaceTouchscreenMenu
from charmie_interfaces.msg import ListOfDetectedPerson, ListOfDetectedObject, TrackingMask
from sensor_msgs.msg import Image as Image_ ### HAD TO CHANGE IMAGE TO IMAGE_ because of: from PIL import Image
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


DEBUG_WITHOUT_DISPLAY = False

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
        # Yolo Pose
        self.person_pose_filtered_subscriber = self.create_subscription(ListOfDetectedPerson, "person_pose_filtered", self.person_pose_filtered_callback, 10)
        # Yolo Objects
        self.objects_filtered_subscriber = self.create_subscription(ListOfDetectedObject, 'objects_all_detected_filtered', self.object_detected_filtered_callback, 10)
        # Tracking (SAM2)
        self.tracking_mask_subscriber = self.create_subscription(TrackingMask, 'tracking_mask', self.tracking_mask_callback, 10)
        
        ### Services (Server) ###   
        self.server_face_command = self.create_service(SetFace, "face_command", self.callback_face_command) 
        self.server_face_set_touchscreen_menu = self.create_service(SetFaceTouchscreenMenu, "set_face_touchscreen_menu", self.callback_face_set_touchscreen_menu) 
        self.face_get_touchscreen_menu_client = self.create_client(GetFaceTouchscreenMenu, "get_face_touchscreen_menu")
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
        self.show_camera_detections = False

        self.list_options_touchscreen_menu = []
        self.is_touchscreen_menu = False
        self.selected_touchscreen_option = []
        self.touchscreen_menu_timeout = 0.0
        self.touchscreen_menu_mode = "single"
        self.touchscreen_menu_start_time = None

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

        self.detected_people = ListOfDetectedPerson()
        self.new_detected_people = False
        self.detected_objects = ListOfDetectedObject()
        self.new_detected_objects = False
        self.tracking_mask = TrackingMask()
        self.new_tracking_mask_msg = False
        self.is_yolo_pose_comm = False
        self.is_yolo_obj_comm = False
        self.is_tracking_comm = False

        self.br = CvBridge()

        self.create_timer(1.0, self.check_yolos_timer)
        self.create_timer(0.4, self.check_tracking_timer)

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
        self.show_camera_detections = False
        if request.command != "":
            response.success, response.message = self.image_to_face(command=request.command)
        elif request.custom != "":
            response.success, response.message = self.custom_image_to_face(command=request.custom)
        elif request.camera != "":
            self.cams_flag = True
            self.selected_camera_stream = request.camera.replace("_"," ").lower()
            print(self.selected_camera_stream)
            self.show_camera_detections = request.show_detections
        else:
            response.success = False
            response.message = "No standard or custom face received."

        print("Face Request:", request.command, request.custom, request.camera, request.show_detections)

        return response

    def call_face_get_touchscreen_menu_server(self, request=GetFaceTouchscreenMenu.Request()):
        self.face_get_touchscreen_menu_client.call_async(request)

    def callback_face_set_touchscreen_menu(self, request, response):

        # Type of service received: 
        # string[] command # options for the face touchscreen menu
        # float64 timeout  # maximum wait time for a menu selection
        # string mode      # select whether we need a single selection or a list of selected items
        # ---
        # bool success   # indicate successful run of triggered service
        # string message # informational, e.g. for error messages.
        
        self.list_options_touchscreen_menu  = request.command
        self.touchscreen_menu_timeout       = request.timeout
        self.touchscreen_menu_mode          = request.mode
        
        self.is_touchscreen_menu = True
        self.selected_touchscreen_option.clear()

        print(request.command)

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
                    
    # CAMERAS 
    def get_rgbd_head_callback(self, rgbd: RGBD):
        self.head_rgb = cv2.cvtColor(self.br.imgmsg_to_cv2(rgbd.rgb, "bgr8"), cv2.COLOR_BGR2RGB)
        self.head_depth = self.get_cv2_cvtColor_from_depth_image(rgbd.depth, "head")
        # print("HEAD:", rgbd.rgb_camera_info.height, rgbd.rgb_camera_info.width, rgbd.depth_camera_info.height, rgbd.depth_camera_info.width)

    def get_rgbd_hand_callback(self, rgbd: RGBD):
        self.hand_rgb = cv2.cvtColor(self.br.imgmsg_to_cv2(rgbd.rgb, "bgr8"), cv2.COLOR_BGR2RGB)
        self.hand_depth = self.get_cv2_cvtColor_from_depth_image(rgbd.depth, "hand")
        # print("HAND:", rgbd.rgb_camera_info.height, rgbd.rgb_camera_info.width, rgbd.depth_camera_info.height, rgbd.depth_camera_info.width)

    def get_color_image_base_callback(self, img: Image):
        self.base_rgb = cv2.cvtColor(self.br.imgmsg_to_cv2(img, "bgr8"), cv2.COLOR_BGR2RGB)
        
    def get_depth_base_image_callback(self, img: Image):
        self.base_depth = self.get_cv2_cvtColor_from_depth_image(img, "base")
    
    # DETECTIONS
    def person_pose_filtered_callback(self, det_people: ListOfDetectedPerson):
        self.detected_people = det_people
        self.new_detected_people = True
        
    def object_detected_filtered_callback(self, det_object: ListOfDetectedObject):
        self.detected_objects = det_object
        self.new_detected_objects = True
        # self.head_yo_time = time.time()
        # self.yolo_objects_fps_ctr += 1
        # for obj in self.detected_objects.objects:
        #     print(obj.object_name, "(", obj.position_cam.x, obj.position_cam.y, obj.position_cam.z, ") (", obj.position_relative.x, obj.position_relative.y, obj.position_relative.z, ") (", obj.position_absolute.x, obj.position_absolute.y, obj.position_absolute.z, ")" )

    def tracking_mask_callback(self, mask: TrackingMask):
        self.tracking_mask = mask
        self.new_tracking_mask_msg = True
        
    def check_yolos_timer(self):
        
        if self.new_detected_people:
            self.new_detected_people = False
            self.is_yolo_pose_comm = True
        else:
            self.is_yolo_pose_comm = False

        if self.new_detected_objects:
            self.new_detected_objects = False
            self.is_yolo_obj_comm = True
        else:
            self.is_yolo_obj_comm = False
    
    def check_tracking_timer(self):

        if self.new_tracking_mask_msg:
            self.new_tracking_mask_msg = False
            self.is_tracking_comm = True
        else:
            self.is_tracking_comm = False


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
            self.resolution = self.get_display_resolution()
            self.display_name = self.get_display_output_name_by_resolution(self.resolution[0], self.resolution[1])
            print("Display Name:", self.display_name)
            self.map_touchscreen_to_correct_display(device_id=self.device_id, display_name=self.display_name)
            self.SCREEN = self.initiliase_pygame_screen(screen=1)
        else:
            self.resolution = [1280, 800]
            self.SCREEN = self.initiliase_pygame_screen(screen=0)
            
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

        self.RED     = (255,  0,  0)
        self.GREEN   = (  0,255,  0)
        self.BLUE    = ( 50, 50,255)
        self.BLUE_L  = (  0,128,255)
        self.WHITE   = (255,255,255)
        self.GREY    = (128,128,128)
        self.BLACK   = (  0,  0,  0)
        self.ORANGE  = (255,153, 51)
        self.MAGENTA = (255, 51,255)
        self.YELLOW  = (255,255,  0)
        self.PURPLE  = (132, 56,255)
        self.CYAN    = (  0,255,255)
        self.GREY_LAR_LOGO = (64, 64, 64)
        self.LIGHT_BLUE_CHARMIE_FACE = (216, 231, 240)
        self.GREEN_PASTEL = (119, 221, 119)
        self.RED_PASTEL = (255, 116, 108)

        # Clear all confirmation-related state
        self.confirming_selection = False
        self.pressed_button_name = None
        self.pressed_confirm_button = None
        self.selected_candidate_name = ""

        # Reset multi-selection state if applicable
        self.confirming_multi_selection = False
        self.pressed_plus_minus = None
        self.pressed_confirm_button = None
        self.option_counts = {}


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
    
    def get_display_output_name_by_resolution(self, target_width, target_height):
        result = subprocess.run(["xrandr"], capture_output=True, text=True)
        for line in result.stdout.splitlines():
            if " connected" in line:
                parts = line.split()
                output_name = parts[0]
                for part in parts:
                    if "x" in part and "+" in part:
                        # print(part)
                        try:
                            res = part.split("+")[0]
                            width, height = map(int, res.split("x"))
                            if width == target_width and height == target_height:
                                return output_name
                        except ValueError:
                            continue
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

    def draw_circle_keypoint(self, surface, conf, x, y, color, min_draw_conf, circle_radius):
        if conf > min_draw_conf:
            pygame.draw.circle(surface, color, (x, y), radius=circle_radius, width=0)
                
    def draw_line_between_two_keypoints(self, surface, conf1, x1, y1, conf2, x2, y2, color, min_draw_conf, min_kp_line_width):
        if conf1 > min_draw_conf and conf2 > min_draw_conf:  
            pygame.draw.line(surface, color, (x1, y1), (x2, y2), min_kp_line_width)
    
    def draw_polygon_alpha(self, surface, color, points):
        lx, ly = zip(*points)
        min_x, min_y, max_x, max_y = min(lx), min(ly), max(lx), max(ly)
        target_rect = pygame.Rect(min_x, min_y, max_x-min_x, max_y-min_y)
        shape_surf = pygame.Surface(target_rect.size, pygame.SRCALPHA)
        pygame.draw.polygon(shape_surf, color, [(x-min_x, y-min_y) for x,y in points])
        surface.blit(shape_surf, target_rect)

    def draw_text(self, surface, text, font, text_col, x, y):
        img = font.render(text, True, text_col)
        surface.blit(img, (x, y))

    def draw_transparent_rect(self, surface, x, y, width, height, color, alpha):
        temp_surface = pygame.Surface((width, height), pygame.SRCALPHA)
        temp_surface.fill((*color, alpha))
        surface.blit(temp_surface, (x, y))

    def object_class_to_bb_color(self, object_class):

        if object_class == "Cleaning Supplies":
            bb_color = self.YELLOW
        elif object_class == "Drinks":
            bb_color = self.PURPLE
        elif object_class == "Foods":
            bb_color = self.BLUE_L
        elif object_class == "Fruits":
            bb_color = self.ORANGE
        elif object_class == "Toys":
            bb_color = self.BLUE
        elif object_class == "Snacks":
            bb_color = self.MAGENTA
        elif object_class == "Dishes":
            bb_color = self.GREY
        elif object_class == "Footwear":
            bb_color = self.RED
        elif object_class == "Furniture":
            bb_color = self.GREEN
        else:
            bb_color = self.BLACK

        return bb_color

    def show_yolo_object_detections(self, surface, camera):
        
        if self.node.is_yolo_obj_comm:

            BB_WIDTH = 3

            for o in self.node.detected_objects.objects:
                
                if o.camera == camera: # checks if camera stream shown is the same as the camera that detected the detection 
                    
                    temp_mask = []
                    for p in o.mask.point: # converts received mask into local coordinates and numpy array
                        p_list = []
                        p_list.append(int(p.x))
                        p_list.append(int(p.y))
                        temp_mask.append(p_list)

                    np_mask = np.array(temp_mask)
                    if len(np_mask) > 2:
                        bb_color = self.object_class_to_bb_color(o.object_class)
                        pygame.draw.polygon(surface, bb_color, np_mask, BB_WIDTH) # outside line (darker)
                        self.draw_polygon_alpha(surface, bb_color+(128,), np_mask) # inside fill with transparecny


            # Removed text from object detection in face, because we had to mirror image, and the text was not readable anymore.
            # In the future we can do the mirror right when we receive the image, so that the text is readable again.
            # However we must mirror all dections as well.
            """ # this is separated into two for loops so that no bounding box overlaps with the name of the object, making the name unreadable 
            text_font_t = pygame.font.SysFont(None, 28)
            for o in self.node.detected_objects.objects:
                
                if o.camera == camera: # checks if camera stream shown is the same as the camera that detected the detection 
                    text = str(o.object_name) # +" ("+str(o.index)+")"
                    text_width, text_height = text_font_t.size(text)
                    bb_color = self.object_class_to_bb_color(o.object_class)

                    if int(o.box_top_left_y) < 30: # depending on the height of the box, so it is either inside or outside
                        self.draw_transparent_rect(surface, int(o.box_top_left_x), int(o.box_top_left_y), text_width, text_height, bb_color, 255)
                        self.draw_text(surface, text, text_font_t, self.BLACK, int(o.box_top_left_x), int(o.box_top_left_y))
                    else:
                        self.draw_transparent_rect(surface, int(o.box_top_left_x), int(o.box_top_left_y-text_height+5), text_width, text_height, bb_color, 255)
                        self.draw_text(surface, text, text_font_t, self.BLACK, int(o.box_top_left_x), int(o.box_top_left_y-text_height+5)) """

    def show_yolo_pose_detections(self, surface, camera):

        if self.node.is_yolo_pose_comm:

            BB_WIDTH = 3
            MIN_DRAW_CONF = 0.5
            CIRCLE_RADIUS = 4
            MIN_KP_LINE_WIDTH = 3

            for p in self.node.detected_people.persons:
        
                if p.camera == camera: # checks if camera stream shown is the same as the camera that detected the detection 
                    
                    ### BOUNDING BOX
                    # PERSON_BB = pygame.Rect(int(p.box_top_left_x), int(p.box_top_left_y), int(p.box_width), int(p.box_height))
                    # pygame.draw.rect(surface, self.RED, PERSON_BB, width=BB_WIDTH)

                    ### LINES BETWEEN KEYPOINTS
                    self.draw_line_between_two_keypoints(surface, p.kp_nose_conf,           p.kp_nose_x,            p.kp_nose_y,            p.kp_eye_left_conf,         p.kp_eye_left_x,        p.kp_eye_left_y,        self.GREEN,   MIN_DRAW_CONF, MIN_KP_LINE_WIDTH)
                    self.draw_line_between_two_keypoints(surface, p.kp_nose_conf,           p.kp_nose_x,            p.kp_nose_y,            p.kp_eye_right_conf,        p.kp_eye_right_x,       p.kp_eye_right_y,       self.GREEN,   MIN_DRAW_CONF, MIN_KP_LINE_WIDTH)
                    self.draw_line_between_two_keypoints(surface, p.kp_eye_left_conf,       p.kp_eye_left_x,        p.kp_eye_left_y,        p.kp_ear_left_conf,         p.kp_ear_left_x,        p.kp_ear_left_y,        self.GREEN,   MIN_DRAW_CONF, MIN_KP_LINE_WIDTH)
                    self.draw_line_between_two_keypoints(surface, p.kp_eye_right_conf,      p.kp_eye_right_x,       p.kp_eye_right_y,       p.kp_ear_right_conf,        p.kp_ear_right_x,       p.kp_ear_right_y,       self.GREEN,   MIN_DRAW_CONF, MIN_KP_LINE_WIDTH)
                    self.draw_line_between_two_keypoints(surface, p.kp_ear_left_conf,       p.kp_ear_left_x,        p.kp_ear_left_y,        p.kp_shoulder_left_conf,    p.kp_shoulder_left_x,   p.kp_shoulder_left_y,   self.GREEN,   MIN_DRAW_CONF, MIN_KP_LINE_WIDTH)
                    self.draw_line_between_two_keypoints(surface, p.kp_ear_right_conf,      p.kp_ear_right_x,       p.kp_ear_right_y,       p.kp_shoulder_right_conf,   p.kp_shoulder_right_x,  p.kp_shoulder_right_y,  self.GREEN,   MIN_DRAW_CONF, MIN_KP_LINE_WIDTH)
                    
                    self.draw_line_between_two_keypoints(surface, p.kp_shoulder_left_conf,  p.kp_shoulder_left_x,   p.kp_shoulder_left_y,   p.kp_shoulder_right_conf,   p.kp_shoulder_right_x,  p.kp_shoulder_right_y,  self.BLUE_L,  MIN_DRAW_CONF, MIN_KP_LINE_WIDTH)
                    self.draw_line_between_two_keypoints(surface, p.kp_shoulder_left_conf,  p.kp_shoulder_left_x,   p.kp_shoulder_left_y,   p.kp_elbow_left_conf,       p.kp_elbow_left_x,      p.kp_elbow_left_y,      self.BLUE_L,  MIN_DRAW_CONF, MIN_KP_LINE_WIDTH)
                    self.draw_line_between_two_keypoints(surface, p.kp_shoulder_right_conf, p.kp_shoulder_right_x,  p.kp_shoulder_right_y,  p.kp_elbow_right_conf,      p.kp_elbow_right_x,     p.kp_elbow_right_y,     self.BLUE_L,  MIN_DRAW_CONF, MIN_KP_LINE_WIDTH)
                    self.draw_line_between_two_keypoints(surface, p.kp_elbow_left_conf,     p.kp_elbow_left_x,      p.kp_elbow_left_y,      p.kp_wrist_left_conf,       p.kp_wrist_left_x,      p.kp_wrist_left_y,      self.BLUE_L,  MIN_DRAW_CONF, MIN_KP_LINE_WIDTH)
                    self.draw_line_between_two_keypoints(surface, p.kp_elbow_right_conf,    p.kp_elbow_right_x,     p.kp_elbow_right_y,     p.kp_wrist_right_conf,      p.kp_wrist_right_x,     p.kp_wrist_right_y,     self.BLUE_L,  MIN_DRAW_CONF, MIN_KP_LINE_WIDTH)
                    
                    self.draw_line_between_two_keypoints(surface, p.kp_shoulder_left_conf,  p.kp_shoulder_left_x,   p.kp_shoulder_left_y,   p.kp_hip_left_conf,         p.kp_hip_left_x,        p.kp_hip_left_y,        self.MAGENTA, MIN_DRAW_CONF, MIN_KP_LINE_WIDTH)
                    self.draw_line_between_two_keypoints(surface, p.kp_shoulder_right_conf, p.kp_shoulder_right_x,  p.kp_shoulder_right_y,  p.kp_hip_right_conf,        p.kp_hip_right_x,       p.kp_hip_right_y,       self.MAGENTA, MIN_DRAW_CONF, MIN_KP_LINE_WIDTH)
                    # self.draw_line_between_two_keypoints(surface, p.kp_shoulder_left_conf,  p.kp_shoulder_left_x,   p.kp_shoulder_left_y,   p.kp_hip_right_conf,        p.kp_hip_right_x,       p.kp_hip_right_y,       self.MAGENTA, MIN_DRAW_CONF, MIN_KP_LINE_WIDTH, camera_height)
                    # self.draw_line_between_two_keypoints(surface, p.kp_shoulder_right_conf, p.kp_shoulder_right_x,  p.kp_shoulder_right_y,  p.kp_hip_left_conf,         p.kp_hip_left_x,        p.kp_hip_left_y,        self.MAGENTA, MIN_DRAW_CONF, MIN_KP_LINE_WIDTH, camera_height)
                    
                    self.draw_line_between_two_keypoints(surface, p.kp_hip_left_conf,       p.kp_hip_left_x,        p.kp_hip_left_y,        p.kp_hip_right_conf,        p.kp_hip_right_x,       p.kp_hip_right_y,       self.ORANGE,  MIN_DRAW_CONF, MIN_KP_LINE_WIDTH)
                    self.draw_line_between_two_keypoints(surface, p.kp_hip_left_conf,       p.kp_hip_left_x,        p.kp_hip_left_y,        p.kp_knee_left_conf,        p.kp_knee_left_x,       p.kp_knee_left_y,       self.ORANGE,  MIN_DRAW_CONF, MIN_KP_LINE_WIDTH)
                    self.draw_line_between_two_keypoints(surface, p.kp_hip_right_conf,      p.kp_hip_right_x,       p.kp_hip_right_y,       p.kp_knee_right_conf,       p.kp_knee_right_x,      p.kp_knee_right_y,      self.ORANGE,  MIN_DRAW_CONF, MIN_KP_LINE_WIDTH)
                    self.draw_line_between_two_keypoints(surface, p.kp_knee_left_conf,      p.kp_knee_left_x,       p.kp_knee_left_y,       p.kp_ankle_left_conf,       p.kp_ankle_left_x,      p.kp_ankle_left_y,      self.ORANGE,  MIN_DRAW_CONF, MIN_KP_LINE_WIDTH)
                    self.draw_line_between_two_keypoints(surface, p.kp_knee_right_conf,     p.kp_knee_right_x,      p.kp_knee_right_y,      p.kp_ankle_right_conf,      p.kp_ankle_right_x,     p.kp_ankle_right_y,     self.ORANGE,  MIN_DRAW_CONF, MIN_KP_LINE_WIDTH)
                
                    ### KEYPOINTS
                    self.draw_circle_keypoint(surface, p.kp_nose_conf,           p.kp_nose_x,            p.kp_nose_y,            self.GREEN,  MIN_DRAW_CONF, CIRCLE_RADIUS)
                    self.draw_circle_keypoint(surface, p.kp_eye_left_conf,       p.kp_eye_left_x,        p.kp_eye_left_y,        self.GREEN,  MIN_DRAW_CONF, CIRCLE_RADIUS)
                    self.draw_circle_keypoint(surface, p.kp_eye_right_conf,      p.kp_eye_right_x,       p.kp_eye_right_y,       self.GREEN,  MIN_DRAW_CONF, CIRCLE_RADIUS)
                    self.draw_circle_keypoint(surface, p.kp_ear_left_conf,       p.kp_ear_left_x,        p.kp_ear_left_y,        self.GREEN,  MIN_DRAW_CONF, CIRCLE_RADIUS)
                    self.draw_circle_keypoint(surface, p.kp_ear_right_conf,      p.kp_ear_right_x,       p.kp_ear_right_y,       self.GREEN,  MIN_DRAW_CONF, CIRCLE_RADIUS)
                    
                    self.draw_circle_keypoint(surface, p.kp_shoulder_left_conf,  p.kp_shoulder_left_x,   p.kp_shoulder_left_y,   self.BLUE_L, MIN_DRAW_CONF, CIRCLE_RADIUS)
                    self.draw_circle_keypoint(surface, p.kp_shoulder_right_conf, p.kp_shoulder_right_x,  p.kp_shoulder_right_y,  self.BLUE_L, MIN_DRAW_CONF, CIRCLE_RADIUS)
                    self.draw_circle_keypoint(surface, p.kp_elbow_left_conf,     p.kp_elbow_left_x,      p.kp_elbow_left_y,      self.BLUE_L, MIN_DRAW_CONF, CIRCLE_RADIUS)
                    self.draw_circle_keypoint(surface, p.kp_elbow_right_conf,    p.kp_elbow_right_x,     p.kp_elbow_right_y,     self.BLUE_L, MIN_DRAW_CONF, CIRCLE_RADIUS)
                    self.draw_circle_keypoint(surface, p.kp_wrist_left_conf,     p.kp_wrist_left_x,      p.kp_wrist_left_y,      self.BLUE_L, MIN_DRAW_CONF, CIRCLE_RADIUS)
                    self.draw_circle_keypoint(surface, p.kp_wrist_right_conf,    p.kp_wrist_right_x,     p.kp_wrist_right_y,     self.BLUE_L, MIN_DRAW_CONF, CIRCLE_RADIUS)
                    
                    self.draw_circle_keypoint(surface, p.kp_hip_left_conf,       p.kp_hip_left_x,        p.kp_hip_left_y,        self.ORANGE, MIN_DRAW_CONF, CIRCLE_RADIUS)
                    self.draw_circle_keypoint(surface, p.kp_hip_right_conf,      p.kp_hip_right_x,       p.kp_hip_right_y,       self.ORANGE, MIN_DRAW_CONF, CIRCLE_RADIUS)
                    self.draw_circle_keypoint(surface, p.kp_knee_left_conf,      p.kp_knee_left_x,       p.kp_knee_left_y,       self.ORANGE, MIN_DRAW_CONF, CIRCLE_RADIUS)
                    self.draw_circle_keypoint(surface, p.kp_knee_right_conf,     p.kp_knee_right_x,      p.kp_knee_right_y,      self.ORANGE, MIN_DRAW_CONF, CIRCLE_RADIUS)
                    self.draw_circle_keypoint(surface, p.kp_ankle_left_conf,     p.kp_ankle_left_x,      p.kp_ankle_left_y,      self.ORANGE, MIN_DRAW_CONF, CIRCLE_RADIUS)
                    self.draw_circle_keypoint(surface, p.kp_ankle_right_conf,    p.kp_ankle_right_x,     p.kp_ankle_right_y,     self.ORANGE, MIN_DRAW_CONF, CIRCLE_RADIUS)
                    
                    # add special keypoints for torso and head filtered pixel
                    self.draw_circle_keypoint(surface, 1.0, p.body_center_x, p.body_center_y, self.BLACK, 0.0, 7)
                    self.draw_circle_keypoint(surface, 1.0, p.body_center_x, p.body_center_y, self.RED,   0.0, 4)
                    self.draw_circle_keypoint(surface, 1.0, p.head_center_x, p.head_center_y, self.BLACK, 0.0, 7)
                    self.draw_circle_keypoint(surface, 1.0, p.head_center_x, p.head_center_y, self.RED,   0.0, 4)

    def show_tracking_detections(self, surface, camera):

        if self.node.is_tracking_comm:
           
            BB_WIDTH = 3

            for used_point in self.node.tracking_mask.mask.masks:

                temp_mask = []
                for p in used_point.point: # converts received mask into local coordinates and numpy array
                    p_list = []
                    p_list.append(int(p.x))
                    p_list.append(int(p.y))
                    temp_mask.append(p_list)
                
                np_mask = np.array(temp_mask)
                if len(np_mask) > 2:
                    bb_color = self.WHITE
                    pygame.draw.polygon(surface, bb_color, np_mask, BB_WIDTH) # outside line (darker)
                    self.draw_polygon_alpha(surface, bb_color+(128,), np_mask) # inside fill with transparecny

            self.draw_circle_keypoint(surface, 1.0, self.node.tracking_mask.centroid.x, self.node.tracking_mask.centroid.y, self.BLACK, 0.0, 7)
            self.draw_circle_keypoint(surface, 1.0, self.node.tracking_mask.centroid.x, self.node.tracking_mask.centroid.y, self.WHITE, 0.0, 4)
        
    def handle_touchscreen_menu(self):

        if not hasattr(self, "pressed_button_name"):
            self.pressed_button_name = None
        if not hasattr(self, "pressed_confirm_button"):
            self.pressed_confirm_button = None
        if not hasattr(self, "confirming_selection"):
            self.confirming_selection = False
        if not hasattr(self, "selected_candidate_name"):
            self.selected_candidate_name = ""

        ### 1) Show Selection Grid ###
        if not self.confirming_selection:
            self.SCREEN.fill(self.LIGHT_BLUE_CHARMIE_FACE)  # Light blue background

            num_options = len(self.node.list_options_touchscreen_menu)
            margin = 20 if num_options <= 21 else 6
            num_columns = 3
            button_height = 80 if num_options <= 21 else 54  # Reduce height for 4 columns
            button_width = (self.resolution[0] - (num_columns + 1) * margin) // num_columns

            font_size = 50 if num_columns == 3 else 36
            font = pygame.font.SysFont(None, font_size)

            num_rows = (num_options + num_columns - 1) // num_columns
            total_height = num_rows * (button_height + margin)
            start_y = (self.resolution[1] - total_height) // 2

            self.menu_buttons = []

            for i, name in enumerate(self.node.list_options_touchscreen_menu):
                col = i % num_columns
                row = i // num_columns

                x = margin + col * (button_width + margin)
                y = start_y + row * (button_height + margin)

                rect = pygame.Rect(x, y, button_width, button_height)
                self.menu_buttons.append((rect, name))

                # Visual feedback if being pressed
                if name == self.pressed_button_name:
                    color = (
                        max(self.GREY_LAR_LOGO[0] - 30, 0),
                        max(self.GREY_LAR_LOGO[1] - 30, 0),
                        max(self.GREY_LAR_LOGO[2] - 30, 0),
                    )
                else:
                    color = self.GREY_LAR_LOGO

                pygame.draw.rect(self.SCREEN, color, rect, border_radius=10)

                text_surface = font.render(name, True, self.LIGHT_BLUE_CHARMIE_FACE)
                text_rect = text_surface.get_rect(center=rect.center)
                self.SCREEN.blit(text_surface, text_rect)

            pygame.display.update()

            for event in pygame.event.get():
                if event.type == pygame.MOUSEBUTTONDOWN:
                    pos = pygame.mouse.get_pos()
                    for rect, name in self.menu_buttons:
                        if rect.collidepoint(pos):
                            self.pressed_button_name = name

                elif event.type == pygame.MOUSEBUTTONUP:
                    pos = pygame.mouse.get_pos()
                    for rect, name in self.menu_buttons:
                        if rect.collidepoint(pos) and name == self.pressed_button_name:
                            self.selected_candidate_name = name
                            self.confirming_selection = True
                            self.node.touchscreen_menu_start_time = None
                            print(f"User tapped: {name}")
                    self.pressed_button_name = None

        ### 2) Show Confirmation Screen ###
        else: # if self.confirming_selection:
            self.SCREEN.fill(self.LIGHT_BLUE_CHARMIE_FACE)

            font = pygame.font.SysFont(None, 50)

            # Display selected name
            big_font = pygame.font.SysFont(None, 80)
            label1 = big_font.render("Selected:", True, self.GREY_LAR_LOGO)
            label1_rect = label1.get_rect(center=(self.resolution[0] // 2, self.resolution[1] // 2.5 - 40))
            self.SCREEN.blit(label1, label1_rect)
            label2 = big_font.render(self.selected_candidate_name, True, self.GREY_LAR_LOGO)
            label2_rect = label2.get_rect(center=(self.resolution[0] // 2, self.resolution[1] // 2.5 + 40))
            self.SCREEN.blit(label2, label2_rect)

            # Accept and Decline buttons
            button_width = 300
            button_height = 80
            margin = 40
            center_x = self.resolution[0] // 2
            y = int(self.resolution[1] * 0.60)

            self.accept_button = pygame.Rect(center_x - button_width - margin // 2, y, button_width, button_height)
            self.decline_button = pygame.Rect(center_x + margin // 2, y, button_width, button_height)

            # --- Visual feedback on press ---
            if self.pressed_confirm_button == "accept":
                accept_color = (
                    max(self.GREEN_PASTEL[0] - 30, 0),
                    max(self.GREEN_PASTEL[1] - 30, 0),
                    max(self.GREEN_PASTEL[2] - 30, 0),
                )
            else:
                accept_color = self.GREEN_PASTEL

            if self.pressed_confirm_button == "decline":
                decline_color = (
                    max(self.RED_PASTEL[0] - 30, 0),
                    max(self.RED_PASTEL[1] - 30, 0),
                    max(self.RED_PASTEL[2] - 30, 0),
                )
            else:
                decline_color = self.RED_PASTEL

            pygame.draw.rect(self.SCREEN, accept_color, self.accept_button, border_radius=10)
            pygame.draw.rect(self.SCREEN, decline_color, self.decline_button, border_radius=10)

            accept_text = font.render("Accept", True, self.GREY_LAR_LOGO)
            decline_text = font.render("Decline", True, self.GREY_LAR_LOGO)

            self.SCREEN.blit(accept_text, accept_text.get_rect(center=self.accept_button.center))
            self.SCREEN.blit(decline_text, decline_text.get_rect(center=self.decline_button.center))

            pygame.display.update()

            for event in pygame.event.get():
                if event.type == pygame.MOUSEBUTTONDOWN:
                    pos = pygame.mouse.get_pos()
                    if self.accept_button.collidepoint(pos):
                        self.pressed_confirm_button = "accept"
                    elif self.decline_button.collidepoint(pos):
                        self.pressed_confirm_button = "decline"

                elif event.type == pygame.MOUSEBUTTONUP:
                    pos = pygame.mouse.get_pos()
                    if self.accept_button.collidepoint(pos) and self.pressed_confirm_button == "accept":
                        self.node.selected_touchscreen_option.append(self.selected_candidate_name) 
                        self.node.is_touchscreen_menu = False
                        self.confirming_selection = False
                        self.node.touchscreen_menu_start_time = None
                        print(f"User accepted: {self.selected_candidate_name}")
            
                        # send selected items through service
                        request = GetFaceTouchscreenMenu.Request()
                        request.command = self.node.selected_touchscreen_option
                        self.node.call_face_get_touchscreen_menu_server(request=request)

                    elif self.decline_button.collidepoint(pos) and self.pressed_confirm_button == "decline":
                        self.confirming_selection = False
                        self.pressed_button_name = None
                        self.selected_candidate_name = ""
                        self.node.touchscreen_menu_start_time = None
                        print("User declined selection")

                    self.pressed_confirm_button = None  # Always reset
            return
                
    def handle_touchscreen_menu_multiple_options(self):
        
        if not hasattr(self, "option_counts") or not self.option_counts:
            self.option_counts = {name: 0 for name in self.node.list_options_touchscreen_menu}
        if not hasattr(self, "pressed_plus_minus"):
            self.pressed_plus_minus = None
        if not hasattr(self, "confirming_multi_selection"):
            self.confirming_multi_selection = False
        if not hasattr(self, "pressed_confirm_button"):
            self.pressed_confirm_button = None
        if not hasattr(self, "confirming_multi_button"):
            self.confirming_multi_button = None


        ### 1) Show Selection Grid ###
        if not self.confirming_multi_selection:

            self.SCREEN.fill(self.LIGHT_BLUE_CHARMIE_FACE)

            num_options = len(self.node.list_options_touchscreen_menu)
            margin = 20 if num_options <= 21 else 6
            num_columns = 3 # 4 if num_options > 21 else 3
            button_height = 80 if num_options <= 21 else 54  # Reduce height for 4 columns
            button_width = (self.resolution[0] - (num_columns + 1) * margin) // num_columns

            font_size = 50 if num_columns == 3 else 36
            font = pygame.font.SysFont(None, font_size)
            
            num_rows = (num_options + num_columns - 1) // num_columns
            total_height = num_rows * (button_height + margin)
            start_y = (self.resolution[1] - total_height) // 2

            self.plus_buttons = []
            self.minus_buttons = []

            for i, name in enumerate(self.node.list_options_touchscreen_menu):
                col = i % num_columns
                row = i // num_columns

                x = margin + col * (button_width + margin)
                y = start_y + row * (button_height + margin)

                main_rect = pygame.Rect(x, y, button_width, button_height)
                pygame.draw.rect(self.SCREEN, self.GREY_LAR_LOGO, main_rect, border_radius=10)

                name_font = pygame.font.SysFont(None, 40) if len(name) > 13 else font
                
                text_surface = name_font.render(name, True, self.LIGHT_BLUE_CHARMIE_FACE)
                text_rect = text_surface.get_rect(midleft=(main_rect.left + 10, main_rect.centery))
                self.SCREEN.blit(text_surface, text_rect)

                spacing = 20
                button_size = 40
                button_center_y = y + button_height // 2

                plus_rect = pygame.Rect(main_rect.right - button_size - 10, button_center_y - button_size // 2, button_size, button_size)
                minus_rect = pygame.Rect(plus_rect.left - spacing - button_size - spacing, button_center_y - button_size // 2, button_size, button_size)

                self.plus_buttons.append((plus_rect, name))
                self.minus_buttons.append((minus_rect, name))

                is_plus_pressed = self.pressed_plus_minus == ('+', name)
                is_minus_pressed = self.pressed_plus_minus == ('-', name)

                plus_color = tuple(max(c - 30, 0) for c in self.LIGHT_BLUE_CHARMIE_FACE) if is_plus_pressed else self.LIGHT_BLUE_CHARMIE_FACE
                minus_color = tuple(max(c - 30, 0) for c in self.LIGHT_BLUE_CHARMIE_FACE) if is_minus_pressed else self.LIGHT_BLUE_CHARMIE_FACE

                pygame.draw.rect(self.SCREEN, minus_color, minus_rect, border_radius=5)
                pygame.draw.rect(self.SCREEN, plus_color, plus_rect, border_radius=5)

                minus_text = font.render("-", True, self.GREY_LAR_LOGO)
                plus_text = font.render("+", True, self.GREY_LAR_LOGO)

                minus_text_rect = minus_text.get_rect(center=minus_rect.center)
                plus_text_rect = plus_text.get_rect(center=plus_rect.center)
                plus_text_rect.y -= 3
                self.SCREEN.blit(minus_text, minus_text_rect)
                self.SCREEN.blit(plus_text, plus_text_rect)

                count = self.option_counts.get(name, 0)
                count_surface = font.render(str(count), True, self.LIGHT_BLUE_CHARMIE_FACE)
                count_rect = count_surface.get_rect()
                count_center_x = (minus_rect.right + plus_rect.left) // 2
                count_rect.center = (count_center_x, button_center_y)
                self.SCREEN.blit(count_surface, count_rect)

            confirm_rect = pygame.Rect(self.resolution[0] // 2 - 125, self.resolution[1] - 100, 250, 70)
            is_confirm_pressed = self.pressed_plus_minus == ('confirm', '')
            confirm_color = tuple(max(c - 30, 0) for c in self.GREEN_PASTEL) if is_confirm_pressed else self.GREEN_PASTEL

            pygame.draw.rect(self.SCREEN, confirm_color, confirm_rect, border_radius=10)
            self.SCREEN.blit(font.render("Confirm", True, self.GREY_LAR_LOGO), font.render("Confirm", True, self.GREY_LAR_LOGO).get_rect(center=confirm_rect.center))

            pygame.display.update()

            for event in pygame.event.get():
                if event.type == pygame.MOUSEBUTTONDOWN:
                    pos = pygame.mouse.get_pos()
                    for rect, name in self.plus_buttons:
                        if rect.collidepoint(pos):
                            self.pressed_plus_minus = ('+', name)
                    for rect, name in self.minus_buttons:
                        if rect.collidepoint(pos):
                            self.pressed_plus_minus = ('-', name)
                    if confirm_rect.collidepoint(pos):
                        self.pressed_plus_minus = ('confirm', '')

                elif event.type == pygame.MOUSEBUTTONUP:
                    pos = pygame.mouse.get_pos()
                    if self.pressed_plus_minus:
                        symbol, name = self.pressed_plus_minus
                        if symbol == '+':
                            for rect, rname in self.plus_buttons:
                                if rname == name and rect.collidepoint(pos):
                                    self.node.touchscreen_menu_start_time = None
                                    if self.option_counts[name] < 9:
                                        self.option_counts[name] += 1
                        elif symbol == '-':
                            for rect, rname in self.minus_buttons:
                                if rname == name and rect.collidepoint(pos):
                                    self.node.touchscreen_menu_start_time = None
                                    if self.option_counts[name] > 0:
                                        self.option_counts[name] -= 1
                        elif symbol == 'confirm':
                            if confirm_rect.collidepoint(pos):
                                self.confirming_multi_selection = True
                                self.node.touchscreen_menu_start_time = None
                        self.pressed_plus_minus = None


        ### 2) Show Confirmation Screen ###
        else: # if self.confirming_multi_selection:
            self.SCREEN.fill(self.LIGHT_BLUE_CHARMIE_FACE)

            font = pygame.font.SysFont(None, 50)

            big_font = pygame.font.SysFont(None, 80)
            label = big_font.render("Confirm Selection", True, self.GREY_LAR_LOGO)
            label_rect = label.get_rect(center=(self.resolution[0] // 2, 100))
            self.SCREEN.blit(label, label_rect)

            font_items = pygame.font.SysFont(None, 80)
            selected = [(name, count) for name, count in self.option_counts.items() if count > 0]
            for i, (name, count) in enumerate(selected):
                line = f"{count}x {name}"
                line_surface = font_items.render(line, True, self.GREY_LAR_LOGO)
                self.SCREEN.blit(line_surface, (100, 160 + i * 50))

            button_width = 300
            button_height = 80
            margin = 40
            center_x = self.resolution[0] // 2
            y = self.resolution[1] - 150

            self.accept_button = pygame.Rect(center_x - button_width - margin // 2, y, button_width, button_height)
            self.decline_button = pygame.Rect(center_x + margin // 2, y, button_width, button_height)

            accept_color = tuple(max(c - 30, 0) for c in self.GREEN_PASTEL) if self.confirming_multi_button == "accept" else self.GREEN_PASTEL
            decline_color = tuple(max(c - 30, 0) for c in self.RED_PASTEL) if self.confirming_multi_button == "decline" else self.RED_PASTEL

            pygame.draw.rect(self.SCREEN, accept_color, self.accept_button, border_radius=10)
            pygame.draw.rect(self.SCREEN, decline_color, self.decline_button, border_radius=10)

            accept_text = font.render("Accept", True, self.GREY_LAR_LOGO)
            decline_text = font.render("Decline", True, self.GREY_LAR_LOGO)

            self.SCREEN.blit(accept_text, accept_text.get_rect(center=self.accept_button.center))
            self.SCREEN.blit(decline_text, decline_text.get_rect(center=self.decline_button.center))

            pygame.display.update()

            for event in pygame.event.get():
                if event.type == pygame.MOUSEBUTTONDOWN:
                    pos = pygame.mouse.get_pos()
                    if self.accept_button.collidepoint(pos):
                        self.confirming_multi_button = "accept"
                    elif self.decline_button.collidepoint(pos):
                        self.confirming_multi_button = "decline"

                elif event.type == pygame.MOUSEBUTTONUP:
                    pos = pygame.mouse.get_pos()
                    if self.accept_button.collidepoint(pos) and self.confirming_multi_button == "accept":
                        result_list = []
                        for name, count in self.option_counts.items():
                            result_list.extend([name] * count)
                        print("Confirmed selection:", result_list)

                        request = GetFaceTouchscreenMenu.Request()
                        request.command = result_list
                        self.node.call_face_get_touchscreen_menu_server(request=request)

                        self.node.is_touchscreen_menu = False
                        self.node.touchscreen_menu_start_time = None
                        self.option_counts = {}
                        self.confirming_multi_selection = False

                    elif self.decline_button.collidepoint(pos) and self.confirming_multi_button == "decline":
                        self.confirming_multi_selection = False
                        self.pressed_plus_minus = None
                        self.confirming_multi_button = None
                        self.option_counts = {}
                        self.node.touchscreen_menu_start_time = None

                    self.confirming_multi_button = None
            return


    def main(self):
        
        while self.running:
            
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False
            
            if self.node.new_text_received:
                self.SCREEN.fill(self.LIGHT_BLUE_CHARMIE_FACE)  # clear screen (black background)
                if self.node.new_text_received_name != "":
                    self.add_text_to_face()
                    pygame.display.update()
                else: # == ""
                    start_time = time.time()
                    self.node.new_text_received = False
                    while time.time() - start_time < self.node.new_text_received_delay:
                        pass # stales the display thread, so that new faces may be received, however the text has higher priority
            
            elif self.node.is_touchscreen_menu:

                if self.node.touchscreen_menu_start_time is None:
                    self.node.touchscreen_menu_start_time = time.time()

                # Check for timeout
                if time.time() - self.node.touchscreen_menu_start_time > self.node.touchscreen_menu_timeout:
                    print("Touchscreen menu timed out. Returning to previous face.")
                    self.node.is_touchscreen_menu = False
                    self.node.touchscreen_menu_start_time = None

                    # Create and send the service request indicating timeout
                    request = GetFaceTouchscreenMenu.Request()
                    request.command = ["TIMEOUT"]
                    self.node.call_face_get_touchscreen_menu_server(request=request)

                    # Clear all confirmation-related state
                    self.confirming_selection = False
                    self.pressed_button_name = None
                    self.pressed_confirm_button = None
                    self.selected_candidate_name = ""

                    # Reset multi-selection state if applicable
                    self.confirming_multi_selection = False
                    self.pressed_plus_minus = None
                    self.pressed_confirm_button = None
                    self.option_counts = {}
                else:

                    if self.node.touchscreen_menu_mode == "single":
                        self.handle_touchscreen_menu()
                    else: # "multi"
                        self.handle_touchscreen_menu_multiple_options()

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
                    
                    if self.node.show_camera_detections:
                        if self.node.selected_camera_stream.split(' ')[0] == "head": # just for head since yolo_pose and tracking are only considered for head, if this changes in the future must edit this
                            self.show_yolo_pose_detections(surface, self.node.selected_camera_stream.split(' ')[0]) # ignores the " depth" when is using depth images
                            self.show_tracking_detections( surface, self.node.selected_camera_stream.split(' ')[0]) # ignores the " depth" when is using depth images
                        self.show_yolo_object_detections(surface, self.node.selected_camera_stream.split(' ')[0])

                    scaled_surface = pygame.transform.scale(surface, self.dynamic_image_resize(surface))
                    # The image used is a copy from GUI, where the prespective was seen from behind the robot,
                    # However when people are looking to the robot face, should seen from front
                    # Otherwise if you move left or right, the image will move in the opposite direction
                    # Thus, we need to flip the image horizontally
                    scaled_surface = pygame.transform.flip(scaled_surface, True, False) # flip the image horizontally
                    self.SCREEN.fill((0, 0, 0))  # cleans display to make sure if the new image does not use all pixels you can not see the pixels from last image on non-used pixels
                    self.SCREEN.blit(scaled_surface, (self.xx_shift, self.yy_shift))
                    pygame.display.update()

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
