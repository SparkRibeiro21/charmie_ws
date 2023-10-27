#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, String, Float32
from charmie_interfaces.msg import SpeechType, RobotSpeech, DetectedPerson, Yolov8Pose
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2


# TO DO TIAGO RIBEIRO:
# - crop face from color_image according to yolo pose  
# - crop hands from color_image according to yolo pose  
# - crop feet from color_image according to yolo pose  
# - sincronizar imagem da camara com imagem do yolo pose 
# - filtro imagens fora do ecra


class PersonRec():
    def __init__(self):
        print("New Person Recognition Class Initialised")

    # your code here

class PersonRecognitionNode(Node):

    def __init__(self):
        super().__init__("PersonRecognition")
        self.get_logger().info("Initialised CHARMIE Person Recognition Node")

        # self.speaker_publisher = self.create_publisher(RobotSpeech, "speech_command", 10)        
        # self.flag_speaker_subscriber = self.create_subscription(Bool, "flag_speech_done", self.get_speech_done_callback, 10)
        
        # image and pose subscriptions
        self.color_image_subscriber = self.create_subscription(Image, "/color/image_raw", self.get_color_image_callback, 10)
        self.person_pose_filtered_subscriber = self.create_subscription(Yolov8Pose, "person_pose_filtered", self.get_person_pose_filtered_callback, 10)   
        
        
        self.create_timer(2, self.check_person_feet)


        self.robot = PersonRec()


        self.latest_color_image = Image()
        self.latest_person_pose = Yolov8Pose()
        self.br = CvBridge()

    # def get_speech_done_callback(self, state: Bool):
    #     print("Received Speech Flag:", state.data)
    #     self.get_logger().info("Received Speech Flag")

    def get_color_image_callback(self, img: Image):
        self.latest_color_image = img
        

    def get_person_pose_filtered_callback(self, pose: Yolov8Pose):
        self.latest_person_pose = pose


    def check_person_face(self):
        
        # ROS2 Image Bridge for OpenCV
        current_frame = self.br.imgmsg_to_cv2(self.latest_color_image, "bgr8")
        current_pose = self.latest_person_pose

        # Crop the image to the rectangle
        # cropped_image = current_frame[100:600, 100:400]

        # Save the cropped image to a file
        # cv2.imwrite("cropped_image.jpg", cropped_image)

        print(current_pose.num_person)

        ctr = 0
        for person in current_pose.persons:
            ctr+=1

            # y1 = topo bounding box y
            # y2 =  maior y dos dois ombros
            # x1 = ombro mais a esq
            # x2 = ombro mais a direita
            
            y1 = person.box_top_left_y
            y2 = min(person.kp_shoulder_right_y, person.kp_shoulder_left_y)


            x1 = min(person.kp_shoulder_right_x, person.kp_shoulder_left_x)
            x2 = max(person.kp_shoulder_right_x, person.kp_shoulder_left_x)

            print(y1, y2, person.kp_shoulder_left_y, " - ", x1, x2)

            # Crop the image to the rectangle
            cropped_image = current_frame[y1:y2, x1:x2]

            try:
                # Save the cropped image to a file
                cv2.imwrite("cropped_face_"+str(ctr)+".jpg", cropped_image)
            except:
                print("An exception has occurred!")

            print(person.conf_person)
                  

    def check_person_hands(self):
        
        # ROS2 Image Bridge for OpenCV
        current_frame = self.br.imgmsg_to_cv2(self.latest_color_image, "bgr8")
        current_pose = self.latest_person_pose

        # Crop the image to the rectangle
        # cropped_image = current_frame[100:600, 100:400]

        # Save the cropped image to a file
        # cv2.imwrite("cropped_image.jpg", cropped_image)

        print(current_pose.num_person)

        ctr = 0
        for person in current_pose.persons:
            ctr+=1

            # y1 = topo bounding box y
            # y2 =  maior y dos dois ombros
            # x1 = ombro mais a esq
            # x2 = ombro mais a direita
            
            # int32 kp_wrist_left_x
            # int32 kp_wrist_left_y
            # float32 kp_wrist_left_conf

            # int32 kp_wrist_right_x
            # int32 kp_wrist_right_y
            # float32 kp_wrist_right_caonf
            
            threshold = 50

            y1_l = person.kp_wrist_left_y - threshold
            y2_l = person.kp_wrist_left_y + threshold

            x1_l = person.kp_wrist_left_x - threshold
            x2_l = person.kp_wrist_left_x + threshold


            y1_r = person.kp_wrist_right_y - threshold
            y2_r = person.kp_wrist_right_y + threshold

            x1_r = person.kp_wrist_right_x - threshold
            x2_r = person.kp_wrist_right_x + threshold


            # Crop the image to the rectangle
            cropped_image_l = current_frame[y1_l:y2_l, x1_l:x2_l]
            cropped_image_r = current_frame[y1_r:y2_r, x1_r:x2_r]

            try:
                # Save the cropped image to a file
                cv2.imwrite("cropped_hand_left_"+str(ctr)+".jpg", cropped_image_l)
                cv2.imwrite("cropped_hand_right_"+str(ctr)+".jpg", cropped_image_r)
            except:
                print("An exception has occurred!")

            print(person.conf_person)
    

    def check_person_feet(self):
        
        # ROS2 Image Bridge for OpenCV
        current_frame = self.br.imgmsg_to_cv2(self.latest_color_image, "bgr8")
        current_pose = self.latest_person_pose

        # Crop the image to the rectangle
        # cropped_image = current_frame[100:600, 100:400]

        # Save the cropped image to a file
        # cv2.imwrite("cropped_image.jpg", cropped_image)

        print(current_pose.num_person)

        ctr = 0
        for person in current_pose.persons:
            ctr+=1

            # y1 = topo bounding box y
            # y2 =  maior y dos dois ombros
            # x1 = ombro mais a esq
            # x2 = ombro mais a direita
            
            # int32 kp_wrist_left_x
            # int32 kp_wrist_left_y
            # float32 kp_wrist_left_conf

            # int32 kp_wrist_right_x
            # int32 kp_wrist_right_y
            # float32 kp_wrist_right_conf

            threshold = 50

            y1_l = person.kp_ankle_left_y - threshold
            y2_l = person.kp_ankle_left_y + threshold

            x1_l = person.kp_ankle_left_x - threshold
            x2_l = person.kp_ankle_left_x + threshold


            y1_r = person.kp_ankle_right_y - threshold
            y2_r = person.kp_ankle_right_y + threshold

            x1_r = person.kp_ankle_right_x - threshold
            x2_r = person.kp_ankle_right_x + threshold


            # Crop the image to the rectangle
            cropped_image_l = current_frame[y1_l:y2_l, x1_l:x2_l]
            cropped_image_r = current_frame[y1_r:y2_r, x1_r:x2_r]

            try:
                # Save the cropped image to a file
                cv2.imwrite("cropped_foot_left_"+str(ctr)+".jpg", cropped_image_l)
                cv2.imwrite("cropped_foot_right_"+str(ctr)+".jpg", cropped_image_r)
            except:
                print("An exception has occurred!")

            print(person.conf_person)


    def check_person_garbage_nearby(self):
        
        # ROS2 Image Bridge for OpenCV
        current_frame = self.br.imgmsg_to_cv2(self.latest_color_image, "bgr8")
        current_pose = self.latest_person_pose

        # Crop the image to the rectangle
        # cropped_image = current_frame[100:600, 100:400]

        # Save the cropped image to a file
        # cv2.imwrite("cropped_image.jpg", cropped_image)

        print(current_pose.num_person)

        ctr = 0
        for person in current_pose.persons:
            ctr+=1

            # y1 = topo bounding box y
            # y2 =  maior y dos dois ombros
            # x1 = ombro mais a esq
            # x2 = ombro mais a direita
            
            # int32 kp_wrist_left_x
            # int32 kp_wrist_left_y
            # float32 kp_wrist_left_conf

            # int32 kp_wrist_right_x
            # int32 kp_wrist_right_y
            # float32 kp_wrist_right_conf

            threshold = 50

            y1_l = person.kp_ankle_left_y - threshold
            y2_l = person.kp_ankle_left_y + threshold

            x1_l = person.kp_ankle_left_x - threshold
            x2_l = person.kp_ankle_left_x + threshold


            y1_r = person.kp_ankle_right_y - threshold
            y2_r = person.kp_ankle_right_y + threshold

            x1_r = person.kp_ankle_right_x - threshold
            x2_r = person.kp_ankle_right_x + threshold


            # Crop the image to the rectangle
            cropped_image_l = current_frame[y1_l:y2_l, x1_l:x2_l]
            cropped_image_r = current_frame[y1_r:y2_r, x1_r:x2_r]

            try:
                # Save the cropped image to a file
                cv2.imwrite("cropped_foot_left_"+str(ctr)+".jpg", cropped_image_l)
                cv2.imwrite("cropped_foot_right_"+str(ctr)+".jpg", cropped_image_r)
            except:
                print("An exception has occurred!")

            print(person.conf_person)


        




def main(args=None):
    rclpy.init(args=args)
    node = PersonRecognitionNode()
    rclpy.spin(node)
    rclpy.shutdown()