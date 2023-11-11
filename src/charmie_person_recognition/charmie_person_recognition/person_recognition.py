#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, String, Float32
from geometry_msgs.msg import Pose2D, Point
from sensor_msgs.msg import Image
from charmie_interfaces.msg import NeckPosition, DetectedPerson, Yolov8Pose, ListOfPoints
from cv_bridge import CvBridge, CvBridgeError
import cv2
import time
import threading
import math


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
        
        self.neck_position_publisher = self.create_publisher(NeckPosition, "neck_to_pos", 10)
        self.neck_to_coords_publisher = self.create_publisher(Pose2D, "neck_to_coords", 10)
        
        self.search_for_person_publisher = self.create_publisher(ListOfPoints, "search_for_person_points", 10)
        # self.create_timer(2, self.check_person_feet)


        self.robot = PersonRec()

        self.latest_color_image = Image()
        self.latest_person_pose = Yolov8Pose()
        self.br = CvBridge()


        self.search_for_person_flag = True
        # self.search_for_person()


    def get_color_image_callback(self, img: Image):
        self.latest_color_image = img
        

    def get_person_pose_filtered_callback(self, pose: Yolov8Pose):
        self.latest_person_pose = pose
        # print("IN")

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
    th_main = threading.Thread(target=thread_main_restaurant, args=(node,), daemon=True)
    th_main.start()
    rclpy.spin(node)
    rclpy.shutdown()


def thread_main_restaurant(node: PersonRecognitionNode):
    main = PersonRecognitionMain(node)
    main.main()

class PersonRecognitionMain():

    def __init__(self, node: PersonRecognitionNode):
        self.node = node

    def main(self):
        
        if self.node.search_for_person_flag:
            self.search_for_person()
            self.node.search_for_person_flag = False
        
    def search_for_person(self):
        print("In Search for Person.")

        tetas = [-120, -60, 0, 60, 120]
        imshow_detected_people = True


        total_person_detected = []
        person_detected = []
        total_cropped_people = []
        cropped_people = []
        points = []
        croppeds = []

        # person_detected_full = []
        # points_full = []



        np = NeckPosition()
        np.pan = float(180 - tetas[0])
        np.tilt = float(180)
        self.node.neck_position_publisher.publish(np)
        time.sleep(1.0)


        # teste Neck to coords
        # aux_neck_to_coords = [
        #     (-0.05, -1.0),
        #     (-0.5, -1.0),
        #     (-0.5, -0.2),
        #     (-0.5,  0.2),
        #     (-0.5,  1.0),
        #     ( 0.5,  1.0),
        #     ( 0.5,  0.2),

        #     ( 0.5, -0.2),
        #     ( 0.5, -1.0)
        # ]
        
        # while True:
        #     for n in aux_neck_to_coords:
        #         pose = Pose2D()
        #         pose.x = n[0]
        #         pose.y = n[1]
        #         pose.theta = 180.0
        #         self.node.neck_to_coords_publisher.publish(pose)
        #         time.sleep(3)




        people_ctr = 0
        for t in tetas:
            print("Rotating Neck:", t)
            
            np = NeckPosition()
            np.pan = float(180 - t)
            np.tilt = float(180)
            self.node.neck_position_publisher.publish(np)
            time.sleep(3)
            # print(self.node.latest_person_pose.num_person)





            for people in self.node.latest_person_pose.persons:
                people_ctr+=1
                print(" - ", people.index_person, people.position_absolute.x,people.position_absolute.y, people.position_absolute.z)
                print(" - ", people.index_person, people.position_relative.x,people.position_relative.y, people.position_relative.z)
                aux = (people.position_absolute.x, people.position_absolute.y) 
                person_detected.append(aux)
                # person_detected_full.append(people)
                points.append(aux)
                # points_full.append(people)




                if imshow_detected_people:

                    y1 = people.box_top_left_y
                    y2 = people.box_top_left_y + people.box_height

                    x1 = people.box_top_left_x
                    x2 = people.box_top_left_x + people.box_width

                    print(y1, y1, x1,x2)
                    br = CvBridge()
                    current_frame = br.imgmsg_to_cv2(self.node.latest_color_image, "bgr8")
                    cropped_image = current_frame[y1:y2, x1:x2]
                    cropped_people.append(cropped_image)
                    
                    # try:
                    #     # Save the cropped image to a file
                    #     cv2.imwrite("cropped_foot_left_"+str(ctr)+".jpg", cropped_image_l)
                    #     cv2.imwrite("cropped_foot_right_"+str(ctr)+".jpg", cropped_image_r)
                    # except:
                    #     print("An exception has occurred!")
                    croppeds.append(cropped_image)

            total_person_detected.append(person_detected.copy())
            total_cropped_people.append(cropped_people.copy())
            print("Total number of people detected:", len(person_detected), people_ctr)
            person_detected.clear()          
            cropped_people.clear()              
            # person_detected_full.clear()

        # print(person_detected_full)

        # print(len(cropped_image))
        # for cropped in cropped_people:
        #     cv2.imshow("Detected People", cropped)
        #     cv2.waitKey(1)
        #     time.sleep(5)

        print(total_person_detected)
        print(len(points))



        """
        total_points = []
        points = []
        p1 =  (-1.805,  0.362)
        ### 
        p2 =  (-2.355,  4.552)
        p3 =  (-3.882,  2.830)
        p4 =  (-1.694,  0.217)
        ###
        p5 =  ( 0.630,  2.417)
        p6 =  (-2.560,  4.700)
        ###
        p7 =  ( 0.892,  3.195)
        p8 =  ( 1.866, -0.373)
        ###
        p9 =  ( 1.754, -0.277)
        p10 = ( 0.561, -0.944)

        print(type(p1))

        points.append(p1)
        total_points.append(points.copy())
        points.clear()

        points.append(p2)
        points.append(p3)
        points.append(p4)
        total_points.append(points.copy())
        points.clear()

        points.append(p5)
        points.append(p6)
        total_points.append(points.copy())
        points.clear()

        points.append(p7)
        points.append(p8)
        total_points.append(points.copy())
        points.clear()

        points.append(p9)
        points.append(p10)
        total_points.append(points.copy())
        points.clear()
        

        points.append(p1)
        points.append(p2)
        points.append(p3)
        points.append(p4)
        points.append(p5)
        points.append(p6)
        points.append(p7)
        points.append(p8)
        points.append(p9)
        points.append(p10)




        print(total_points)
        print("\n\n")

        """

       
        filtered_persons = []
        filtered_persons_cropped = []
        for frame in range(len(total_person_detected)):

            if not len(filtered_persons):
                for person in range(len(total_person_detected[frame])):
                    filtered_persons.append(total_person_detected[frame][person])
                    filtered_persons_cropped.append(total_cropped_people[frame][person])
            else:
                for person in range(len(total_person_detected[frame])):
                    same_person_ctr = 0
                    same_person_coords = (0,0)
                    for filtered in range(len(filtered_persons)): #_aux:

                        # print("??? ", total_person_detected[frame][person], filtered_persons[filtered])
                        dist = math.dist(total_person_detected[frame][person], filtered_persons[filtered])
                        # print("person:", person, "filtered:", filtered, "dist:", dist)
                        
                        if dist < 1.0:
                            same_person_ctr+=1
                            same_person_coords = filtered_persons[filtered]
                            same_person_cropped = filtered_persons_cropped[filtered] 
                        
                    if same_person_ctr > 0:
                        
                        # print(same_person_cropped)
                        # print(total_cropped_people[frame][person])
                        # print(len(same_person_cropped), len(total_cropped_people[frame][person]))
                        # print(same_person_cropped.shape[0], same_person_cropped.shape[1])
                        # print(total_cropped_people[frame][person].shape[0], total_cropped_people[frame][person].shape[1])
                        
                        # the same person is the person on the first frame, whereas total_cropped_people[frame][person] is the same person on the second frame
                        if total_cropped_people[frame][person].shape[1] > same_person_cropped.shape[1]:
                            filtered_persons_cropped.remove(same_person_cropped)
                            filtered_persons_cropped.append(total_cropped_people[frame][person])

                    
                        #print(same_person_ctr, same_person_coords, person)
                        filtered_persons.remove(same_person_coords)

                        avg_person = ((total_person_detected[frame][person][0]+same_person_coords[0])/2, (total_person_detected[frame][person][1]+same_person_coords[1])/2)
                        # print(avg_person)
                        filtered_persons.append(avg_person)
                        points.append(avg_person)

                    else:
                        filtered_persons.append(total_person_detected[frame][person])
                        filtered_persons_cropped.append(total_cropped_people[frame][person])


        """
        filtered_persons = []
        for frame in total_person_detected:

            if not len(filtered_persons):
                for person in frame:
                    filtered_persons.append(person)
            else:
                for person in frame:
                    same_person_ctr = 0
                    same_person_coords = (0,0)
                    for filtered in filtered_persons: #_aux:
                        dist = math.dist(person, filtered)
                        # print("person:", person, "filtered:", filtered, "dist:", dist)
                        
                        if dist < 1.0:
                            same_person_ctr+=1
                            same_person_coords = filtered
                        
                    if same_person_ctr > 0:
                         #print(same_person_ctr, same_person_coords, person)
                        filtered_persons.remove(same_person_coords)
                        
                        avg_person = ((person[0]+same_person_coords[0])/2, (person[1]+same_person_coords[1])/2)
                        # print(avg_person)
                        filtered_persons.append(avg_person)
                        points.append(avg_person)

                    else:
                        filtered_persons.append(person)

        """




        # print("---", filtered_persons)

        
        ctr = 0
        for c in croppeds:
            ctr+=1
            cv2.imwrite("Person Detected_"+str(ctr)+".jpg", c)
        ctr = 0
        for c in filtered_persons_cropped:
            ctr+=1
            cv2.imwrite("Person Filtered_"+str(ctr)+".jpg", c)



        print("---", filtered_persons)
        points_to_send = ListOfPoints()
        # for debug, see all points and the average calculations
        for p in points:
        # for p in filtered_persons:
            aux = Point()
            aux.x = float(p[0])
            aux.y = float(p[1])
            aux.z = 0.0
            points_to_send.coords.append(aux)

        # print(points_to_send)
        self.node.search_for_person_publisher.publish(points_to_send)


        for p in filtered_persons:
            pose = Pose2D()
            pose.x = p[0]
            pose.y = p[1]
            pose.theta = 180.0
            self.node.neck_to_coords_publisher.publish(pose)
            time.sleep(3)

        

        # np = NeckPosition()
        # np.pan = float(180)
        # np.tilt = float(180)
        # self.node.neck_position_publisher.publish(np)
        # time.sleep(3)

        # np = NeckPosition()
        # np.pan = float(180+180)
        # np.tilt = float(180)
        # self.node.neck_position_publisher.publish(np)
        # time.sleep(3)

        # np = NeckPosition()
        # np.pan = float(180)
        # np.tilt = float(180)
        # self.node.neck_position_publisher.publish(np)
        # time.sleep(3)
