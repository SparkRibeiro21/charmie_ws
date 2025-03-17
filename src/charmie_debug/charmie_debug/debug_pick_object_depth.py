#!/usr/bin/env python3
import rclpy
import threading
import time
import numpy as np
import cv2
import math
from charmie_interfaces.msg import BoundingBox
from charmie_std_functions.task_ros2_and_std_functions import ROS2TaskNode, RobotStdFunctions

# Constant Variables to ease RGB_MODE coding
RED, GREEN, BLUE, YELLOW, MAGENTA, CYAN, WHITE, ORANGE, PINK, BROWN  = 0, 10, 20, 30, 40, 50, 60, 70, 80, 90
SET_COLOUR, BLINK_LONG, BLINK_QUICK, ROTATE, BREATH, ALTERNATE_QUARTERS, HALF_ROTATE, MOON, BACK_AND_FORTH_4, BACK_AND_FORTH_8  = 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
CLEAR, RAINBOW_ROT, RAINBOW_ALL, POLICE, MOON_2_COLOUR, PORTUGAL_FLAG, FRANCE_FLAG, NETHERLANDS_FLAG = 255, 100, 101, 102, 103, 104, 105, 106

ros2_modules = {
    "charmie_arm":              False,
    "charmie_audio":            False,
    "charmie_face":             False,
    "charmie_head_camera":      False,
    "charmie_hand_camera":      True,
    "charmie_lidar":            False,
    "charmie_llm":              False,
    "charmie_localisation":     False,
    "charmie_low_level":        False,
    "charmie_navigation":       False,
    "charmie_nav2":             False,
    "charmie_neck":             False,
    "charmie_obstacles":        False,
    "charmie_odometry":         False,
    "charmie_point_cloud":      False,
    "charmie_ps4_controller":   False,
    "charmie_speakers":         False,
    "charmie_tracking":         False,
    "charmie_yolo_objects":     False,
    "charmie_yolo_pose":        False,
}

# main function that already creates the thread for the task state machine
def main(args=None):
    rclpy.init(args=args)
    node = ROS2TaskNode(ros2_modules)
    robot = RobotStdFunctions(node)
    th_main = threading.Thread(target=ThreadMainTask, args=(robot,), daemon=True)
    th_main.start()
    rclpy.spin(node)
    rclpy.shutdown()

def ThreadMainTask(robot: RobotStdFunctions):
    main = TaskMain(robot)
    main.main()

class TaskMain():

    def __init__(self, robot: RobotStdFunctions):
        # create a robot instance so use all standard CHARMIE functions
        self.robot = robot

    def main(self):
        Waiting_for_start_button = 0
        Search_for_bag = 1
        Show_rgb_images = 2
        Final_State = 3
        
        # VARS ...
        self.state = Search_for_bag

        self.floor_dist=600
        self.top_bag_dist=350

        print("IN NEW MAIN")

        while True:

            # State Machine
            # State 0 = Initial
            # State 1 = Hand Raising Detect
            # State 2 = Navigation to Person
            # State 3 = Receive Order - Receive Order - Speech
            # State 4 = Receive Order - Listening and Confirm
            # State 5 = Collect Order
            # State 6 = Final Speech

            if self.state == Waiting_for_start_button:
                # print('State 0 = Initial')

                self.state = Search_for_bag

            elif self.state == Search_for_bag:
                #print('State 1 = Search for bag')

                s = False
                while not s:
                    s, cam = self.robot.get_hand_depth_image()
                    if not s:
                        time.sleep(0.1)
                
                bag_coords = self.get_bag_pick_cordinates(current_frame_depth_head=cam) #half_image_zero_or_near_percentage=0.4, full_image_near_percentage=0.1, near_max_dist=800)
                print(bag_coords)
                
            elif self.state == Show_rgb_images:

                while True:
                    self.show_rgb_images()
                    time.sleep(0.1)
            
            elif self.state == Final_State:
                print("Finished task!!!")

                while True:
                    pass

            else:
                pass


    def show_rgb_images(self):

        s1, cam1 = self.robot.get_hand_rgb_image()
        s2, cam2 = self.robot.get_head_rgb_image()

        cv2.imshow("cam1", cam1)
        cv2.imshow("cam2", cam2)

        cv2.waitKey(10)


    def get_bag_pick_cordinates(self, current_frame_depth_head):

        DEBUG = False
        MIN_BAG_PIXEL_AREA = 40000
        f_coords = []

        c_areas = []
        
        while not c_areas or max(c_areas) < MIN_BAG_PIXEL_AREA:
            c_areas.clear()

            s, current_frame_depth_head = self.robot.get_hand_depth_image()
            height, width = current_frame_depth_head.shape
            
            # current_frame_depth_head[int(0.80*height):height,int(0.29*width):int(0.71*width)] = 0 # remove the robot from the image

            mask_zero = (current_frame_depth_head == 0)
            mask_near = (current_frame_depth_head != 0) & (current_frame_depth_head >= self.top_bag_dist) & (current_frame_depth_head <= self.floor_dist)

            blank_image_bw = np.zeros((height,width), np.uint8)
            blank_image_bw2 = np.zeros((height,width), np.uint8)
            blank_image_bw[mask_near] = [255]

            contours, hierarchy = cv2.findContours(blank_image_bw, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

            print("areas")
            for a in contours: # create list with area size 
                print(cv2.contourArea(a))
                c_areas.append(cv2.contourArea(a))
            if c_areas:
                print(max(c_areas), " ...")

        cnt = contours[c_areas.index(max(c_areas))] # extracts the largest area 
        # print(c_areas.index(max(c_areas)))
        
        M = cv2.moments(cnt) # calculates centroide
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])

        xi,yi,w,h = cv2.boundingRect(cnt)

        [vx,vy,x,y] = cv2.fitLine(cnt, cv2.DIST_L2,0,0.01,0.01)
        theta = -math.atan2(vy,vx)

        # bb_thresh = 40 
        # bb = BoundingBox() # centroide of bag bounding box 
        # bb.box_top_left_x = max(cx-bb_thresh, 0)
        # bb.box_top_left_y = max(cy-bb_thresh, 0)
        # if bb.box_top_left_x + 2*bb_thresh > width:
        #     bb.box_width = width - bb.box_top_left_x 
        # else:
        #     bb.box_width = 2*bb_thresh
        # if bb.box_top_left_y + 2*bb_thresh > height:
        #     bb.box_height = height - bb.box_top_left_y 
        # else:
        #     bb.box_height = 2*bb_thresh
        # coords_centroide = self.get_point_cloud(bb=bb)
        
        bb = BoundingBox() # full bag bounding box
        bb.box_top_left_x = max(xi, 0)
        bb.box_top_left_y = max(yi, 0)
        bb.box_width = w
        bb.box_height = h

        coords_full = self.robot.get_point_cloud(bb=bb)
        selected_coords = coords_full

        # adds difference between camera center to gripper center
        selected_coords.center_coords.z += 70

        # x = bag height
        # y = move front and back robot, or left and right for camera
        # z = move right and left robot, or up and down for camera
        # print("xc = ", round(coords_centroide.center_coords.x,0), "yc = ", round(coords_centroide.center_coords.y,0), "zc = ", round(coords_centroide.center_coords.z,0))
        print("xf = ", round(coords_full.center_coords.x,0), "yf = ", round(coords_full.center_coords.y,0), "zf = ", round(coords_full.center_coords.z,0))

        ang_to_bag = -math.degrees(math.atan2(selected_coords.center_coords.z, selected_coords.center_coords.y))
        dist_to_bag = (math.sqrt(selected_coords.center_coords.y**2 + selected_coords.center_coords.z**2))/1000
        
        f_coords.append(selected_coords.center_coords.x/1000)
        f_coords.append(selected_coords.center_coords.y/1000)
        f_coords.append(selected_coords.center_coords.z/1000)
        f_coords.append(ang_to_bag)
        f_coords.append(dist_to_bag)
        f_coords.append(theta)
        
        print(ang_to_bag, dist_to_bag)
        
        if DEBUG:
            mask_remaining = (current_frame_depth_head > self.floor_dist) # just for debug, floor level
            blank_image = np.zeros((height,width,3), np.uint8)
            
            blank_image[mask_zero] = [255,255,255]
            blank_image[mask_near] = [255,0,0]
            blank_image[mask_remaining] = [0,0,255]

            cv2.drawContours(blank_image, [cnt], 0, (0, 255, 0), 3) 
            cv2.drawContours(blank_image_bw2, [cnt], 0, (255), thickness=cv2.FILLED) 
            cv2.circle(blank_image, (cx, cy), 10, (0, 255, 0), -1)
            cv2.circle(blank_image_bw2, (cx, cy), 10, (128), -1)

            cv2.rectangle(blank_image,(xi,yi),(xi+w,yi+h), (255, 0, 255),2)
            cv2.rectangle(blank_image_bw2,(xi,yi),(xi+w,yi+h),(128),2)

            rows,cols = blank_image_bw2.shape[:2]
            lefty = int((-x*vy/vx) + y)
            righty = int(((cols-x)*vy/vx)+y)
            cv2.line(blank_image,(cols-1,righty),(0,lefty),(255, 0, 255),2)
            cv2.line(blank_image_bw2,(cols-1,righty),(0,lefty),(128),2)

            print("bag theta =", round(math.degrees(theta), 2), round(theta,2))
            # print("bag centroide =", cx, cy)

            cv2.rectangle(blank_image, (bb.box_top_left_x, bb.box_top_left_y), (bb.box_top_left_x+bb.box_width, bb.box_top_left_y+bb.box_height), (255, 0, 255),2)
            cv2.rectangle(blank_image_bw2, (bb.box_top_left_x, bb.box_top_left_y), (bb.box_top_left_x+bb.box_width, bb.box_top_left_y+bb.box_height), (128), 2)

            cv2.imshow("New Img Distance Inspection", blank_image)
            cv2.imshow("New Img Distance Inspection BW", blank_image_bw2)

            k = cv2.waitKey(1)
            if k == ord('w'):
                self.floor_dist += 10
            if k == ord('q'):
                self.floor_dist -= 10
            if k == ord('s'):
                self.top_bag_dist += 10
            if k == ord('a'):
                self.top_bag_dist -= 10

            print(self.floor_dist, self.top_bag_dist)

        return f_coords