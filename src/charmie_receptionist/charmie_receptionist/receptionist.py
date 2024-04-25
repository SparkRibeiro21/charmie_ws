#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

# import variables from standard libraries and both messages and services from custom charmie_interfaces
from example_interfaces.msg import Bool, String, Int16
from charmie_interfaces.msg import Yolov8Pose, DetectedPerson, Yolov8Objects, DetectedObject
from charmie_interfaces.srv import SpeechCommand, GetAudio, CalibrateAudio, SetNeckPosition, GetNeckPosition, SetNeckCoordinates, TrackObject, TrackPerson, ActivateYoloPose, ActivateYoloObjects, ArmTrigger

import cv2 
import threading
import time
from cv_bridge import CvBridge

# Constant Variables to ease RGB_MODE coding
RED, GREEN, BLUE, YELLOW, MAGENTA, CYAN, WHITE, ORANGE, PINK, BROWN  = 0, 10, 20, 30, 40, 50, 60, 70, 80, 90
SET_COLOUR, BLINK_LONG, BLINK_QUICK, ROTATE, BREATH, ALTERNATE_QUARTERS, HALF_ROTATE, MOON, BACK_AND_FORTH_4, BACK_AND_FORTH_8  = 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
CLEAR, RAINBOW_ROT, RAINBOW_ALL, POLICE, MOON_2_COLOUR, PORTUGAL_FLAG, FRANCE_FLAG, NETHERLANDS_FLAG = 255, 100, 101, 102, 103, 104, 105, 106


class ReceptionistNode(Node):

    def __init__(self):
        super().__init__("Receptionist")
        self.get_logger().info("Initialised CHARMIE Receptionist Node")

        ### Topics (Publisher and Subscribers) ###   
        # Low Level 
        self.rgb_mode_publisher = self.create_publisher(Int16, "rgb_mode", 10)   
        self.start_button_subscriber = self.create_subscription(Bool, "get_start_button", self.get_start_button_callback, 10)
        self.flag_start_button_publisher = self.create_publisher(Bool, "flag_start_button", 10) 
        # Face
        self.image_to_face_publisher = self.create_publisher(String, "display_image_face", 10)
        self.custom_image_to_face_publisher = self.create_publisher(String, "display_custom_image_face", 10)
        # Yolo Pose
        self.person_pose_filtered_subscriber = self.create_subscription(Yolov8Pose, "person_pose_filtered", self.person_pose_filtered_callback, 10)
        # Yolo Objects
        # self.object_detected_filtered_subscriber = self.create_subscription(Yolov8Objects, "objects_detected_filtered", self.object_detected_filtered_callback, 10)
        # Arm CHARMIE
        # self.arm_command_publisher = self.create_publisher(String, "arm_command", 10)
        # self.arm_finished_movement_subscriber = self.create_subscription(Bool, 'arm_finished_movement', self.arm_finished_movement_callback, 10)

        ### Services (Clients) ###
        # Speakers
        self.speech_command_client = self.create_client(SpeechCommand, "speech_command")
        # Audio
        self.get_audio_client = self.create_client(GetAudio, "audio_command")
        self.calibrate_audio_client = self.create_client(CalibrateAudio, "calibrate_audio")
        # Neck
        self.set_neck_position_client = self.create_client(SetNeckPosition, "neck_to_pos")
        # self.get_neck_position_client = self.create_client(GetNeckPosition, "get_neck_pos")
        self.set_neck_coordinates_client = self.create_client(SetNeckCoordinates, "neck_to_coords")
        self.neck_track_person_client = self.create_client(TrackPerson, "neck_track_person")
        # self.neck_track_object_client = self.create_client(TrackObject, "neck_track_object")
        # Yolos
        self.activate_yolo_pose_client = self.create_client(ActivateYoloPose, "activate_yolo_pose")
        # self.activate_yolo_objects_client = self.create_client(ActivateYoloObjects, "activate_yolo_objects")
        # Arm (CHARMIE)
        # self.arm_trigger_client = self.create_client(ArmTrigger, "arm_trigger")


        # if is necessary to wait for a specific service to be ON, uncomment the two following lines
        # Speakers
        while not self.speech_command_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Speech Command...")

        # Audio
        while not self.get_audio_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Audio Server...")
        while not self.calibrate_audio_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Calibrate Audio Server...")
        # Neck 
        while not self.set_neck_position_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Set Neck Position Command...")
        # while not self.get_neck_position_client.wait_for_service(1.0):
        #     self.get_logger().warn("Waiting for Server Get Neck Position Command...")
        while not self.set_neck_coordinates_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Set Neck Coordinates Command...")
        while not self.neck_track_person_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Set Neck Track Object Command...")
        # while not self.neck_track_object_client.wait_for_service(1.0):
        #     self.get_logger().warn("Waiting for Server Set Neck Track Person Command...")
        # Yolos
        while not self.activate_yolo_pose_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Yolo Pose Activate Command...")
        # while not self.activate_yolo_objects_client.wait_for_service(1.0):
        #     self.get_logger().warn("Waiting for Server Yolo Objects Activate Command...")

        # Arm (CHARMIE)
        # while not self.arm_trigger_client.wait_for_service(1.0):
        #     self.get_logger().warn("Waiting for Server Arm Trigger Command...")
        
        # Variables 
        self.waited_for_end_of_audio = False
        self.waited_for_end_of_calibrate_audio = False
        self.waited_for_end_of_speaking = False
        self.waited_for_end_of_neck_pos = False
        self.waited_for_end_of_neck_coords = False
        self.waited_for_end_of_track_person = False
        # self.waited_for_end_of_track_object = False
        # self.waited_for_end_of_arm = False # not used, but here to be in conformity with other uses

        self.br = CvBridge()
        self.detected_people = Yolov8Pose()
        # self.detected_objects = Yolov8Objects()
        self.start_button_state = False
        # self.arm_ready = True

        # Success and Message confirmations for all set_(something) CHARMIE functions
        self.speech_success = True
        self.speech_message = ""
        self.rgb_success = True
        self.rgb_message = ""
        self.calibrate_audio_success = True
        self.calibrate_audio_message = ""
        self.audio_command = ""
        self.face_success = True
        self.face_message = ""
        self.neck_success = True
        self.neck_message = ""
        self.track_person_success = True
        self.track_person_message = ""
        # self.track_object_success = True
        # self.track_object_message = ""
        self.activate_yolo_pose_success = True
        self.activate_yolo_pose_message = ""
        # self.activate_yolo_objects_success = True
        # self.activate_yolo_objects_message = ""
        self.arm_success = True
        self.arm_message = ""

    def person_pose_filtered_callback(self, det_people: Yolov8Pose):
        self.detected_people = det_people

        # current_frame = self.br.imgmsg_to_cv2(self.detected_people.image_rgb, "bgr8")
        # current_frame_draw = current_frame.copy()
        
        # cv2.imshow("Yolo Pose TR Detection 2", current_frame_draw)
        # cv2.waitKey(10)

    ### LOW LEVEL START BUTTON ###
    def get_start_button_callback(self, state: Bool):
        self.start_button_state = state.data
        # print("Received Start Button:", state.data)

    ### ACTIVATE YOLO POSE SERVER FUNCTIONS ###
    def call_activate_yolo_pose_server(self, activate=True, only_detect_person_legs_visible=False, minimum_person_confidence=0.5, minimum_keypoints_to_detect_person=7, only_detect_person_right_in_front=False, only_detect_person_arm_raised=False, characteristics=False):
        request = ActivateYoloPose.Request()
        request.activate = activate
        request.only_detect_person_legs_visible = only_detect_person_legs_visible
        request.minimum_person_confidence = minimum_person_confidence
        request.minimum_keypoints_to_detect_person = minimum_keypoints_to_detect_person
        request.only_detect_person_arm_raised = only_detect_person_arm_raised
        request.only_detect_person_right_in_front = only_detect_person_right_in_front
        request.characteristics = characteristics

        self.activate_yolo_pose_client.call_async(request)

    #### SPEECH SERVER FUNCTIONS #####
    def call_speech_command_server(self, filename="", command="", quick_voice=False, wait_for_end_of=True, show_in_face=False):
        request = SpeechCommand.Request()
        request.filename = filename
        request.command = command
        request.quick_voice = quick_voice
        request.show_in_face = show_in_face
    
        future = self.speech_command_client.call_async(request)
        # print("Sent Command")

        if wait_for_end_of:
            # future.add_done_callback(partial(self.callback_call_speech_command, a=filename, b=command))
            future.add_done_callback(self.callback_call_speech_command)
        else:
            self.speech_success = True
            self.speech_message = "Wait for answer not needed"

    def callback_call_speech_command(self, future): #, a, b):

        try:
            # in this function the order of the line of codes matter
            # it seems that when using future variables, it creates some type of threading system
            # if the falg raised is here is before the prints, it gets mixed with the main thread code prints
            response = future.result()
            self.get_logger().info(str(response.success) + " - " + str(response.message))
            self.speech_success = response.success
            self.speech_message = response.message
            # time.sleep(3)
            self.waited_for_end_of_speaking = True
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))   


    #### AUDIO SERVER FUNCTIONS #####
    def call_audio_server(self, yes_or_no=False, receptionist=False, gpsr=False, restaurant=False, wait_for_end_of=True):
        request = GetAudio.Request()
        request.yes_or_no = yes_or_no
        request.receptionist = receptionist
        request.gpsr = gpsr
        request.restaurant = restaurant

        future = self.get_audio_client.call_async(request)
        # print("Sent Command")

        if wait_for_end_of:
            future.add_done_callback(self.callback_call_audio)
        else:
            self.track_person_success = True
            self.track_person_message = "Wait for answer not needed"
    
    def callback_call_audio(self, future):

        try:
            # in this function the order of the line of codes matter
            # it seems that when using future variables, it creates some type of threading system
            # if the flag raised is here is before the prints, it gets mixed with the main thread code prints
            response = future.result()
            self.get_logger().info(str(response.command))
            self.audio_command = response.command
            # self.track_object_success = response.success
            # self.track_object_message = response.message
            # time.sleep(3)
            self.waited_for_end_of_audio = True
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))


    def call_calibrate_audio_server(self, wait_for_end_of=True):
        request = CalibrateAudio.Request()

        future = self.calibrate_audio_client.call_async(request)
        # print("Sent Command")

        if wait_for_end_of:
            future.add_done_callback(self.callback_call_calibrate_audio)
        else:
            self.track_person_success = True
            self.track_person_message = "Wait for answer not needed"
    
    def callback_call_calibrate_audio(self, future):

        try:
            # in this function the order of the line of codes matter
            # it seems that when using future variables, it creates some type of threading system
            # if the flag raised is here is before the prints, it gets mixed with the main thread code prints
            response = future.result()
            self.get_logger().info(str(response.success) + " - " + str(response.message))
            self.track_person_success = response.success
            self.track_person_message = response.message
            # self.track_object_success = response.success
            # self.track_object_message = response.message
            # time.sleep(3)
            self.waited_for_end_of_calibrate_audio = True
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))

    #### SET NECK POSITION SERVER FUNCTIONS #####
    def call_neck_position_server(self, position=[0, 0], wait_for_end_of=True):
        request = SetNeckPosition.Request()
        request.pan = float(position[0])
        request.tilt = float(position[1])
        
        future = self.set_neck_position_client.call_async(request)
        # print("Sent Command")

        if wait_for_end_of:
            # future.add_done_callback(partial(self.callback_call_speech_command, a=filename, b=command))
            future.add_done_callback(self.callback_call_set_neck_command)
        else:
            self.neck_success = True
            self.neck_message = "Wait for answer not needed"
    
    def callback_call_set_neck_command(self, future): #, a, b):

        try:
            # in this function the order of the line of codes matter
            # it seems that when using future variables, it creates some type of threading system
            # if the falg raised is here is before the prints, it gets mixed with the main thread code prints
            response = future.result()
            self.get_logger().info(str(response.success) + " - " + str(response.message))
            self.speech_success = response.success
            self.speech_message = response.message
            # time.sleep(3)
            self.waited_for_end_of_neck_pos = True
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))   

    #### SET NECK COORDINATES SERVER FUNCTIONS #####
    def call_neck_coordinates_server(self, x, y, z, tilt, flag, wait_for_end_of=True):
        request = SetNeckCoordinates.Request()
        request.coords.x = float(x)
        request.coords.y = float(y)
        request.coords.z = float(z)
        request.is_tilt = flag
        request.tilt = float(tilt)
        
        future = self.set_neck_coordinates_client.call_async(request)
        # print("Sent Command")

        if wait_for_end_of:
            # future.add_done_callback(partial(self.callback_call_speech_command, a=filename, b=command))
            future.add_done_callback(self.callback_call_set_neck_coords_command)
        else:
            self.neck_success = True
            self.neck_message = "Wait for answer not needed"
    
    def callback_call_set_neck_coords_command(self, future): #, a, b):

        try:
            # in this function the order of the line of codes matter
            # it seems that when using future variables, it creates some type of threading system
            # if the falg raised is here is before the prints, it gets mixed with the main thread code prints
            response = future.result()
            self.get_logger().info(str(response.success) + " - " + str(response.message))
            self.speech_sucecss = response.success
            self.speech_message = response.message
            # time.sleep(3)
            self.waited_for_end_of_neck_coords = True
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))   

    #### NECK SERVER FUNCTIONS #####
    def call_neck_track_person_server(self, person, body_part="Head", wait_for_end_of=True):
        request = TrackPerson.Request()
        request.person = person
        request.body_part = body_part

        future = self.neck_track_person_client.call_async(request)
        # print("Sent Command")

        if wait_for_end_of:
            future.add_done_callback(self.callback_call_neck_track_person)
        else:
            self.track_person_success = True
            self.track_person_message = "Wait for answer not needed"
    
    def callback_call_neck_track_person(self, future):

        try:
            # in this function the order of the line of codes matter
            # it seems that when using future variables, it creates some type of threading system
            # if the falg raised is here is before the prints, it gets mixed with the main thread code prints
            response = future.result()
            self.get_logger().info(str(response.success) + " - " + str(response.message))
            self.track_person_success = response.success
            self.track_person_message = response.message
            # time.sleep(3)
            self.waited_for_end_of_track_person = True
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))


# main function that already creates the thread for the task state machine
def main(args=None):
    rclpy.init(args=args)
    node = ReceptionistNode()
    th_main = threading.Thread(target=ThreadMainReceptionist, args=(node,), daemon=True)
    th_main.start()
    rclpy.spin(node)
    rclpy.shutdown()

def ThreadMainReceptionist(node: ReceptionistNode):
    main = ReceptionistMain(node)
    main.main()

class ReceptionistMain():

    def __init__(self, node: ReceptionistNode):
        # create a node instance so all variables ros related can be acessed
        self.node = node

    def set_speech(self, filename="", command="", quick_voice=False, show_in_face=False, wait_for_end_of=True):

        self.node.call_speech_command_server(filename=filename, command=command, wait_for_end_of=wait_for_end_of, quick_voice=quick_voice, show_in_face=show_in_face)
        
        if wait_for_end_of:
          while not self.node.waited_for_end_of_speaking:
            pass
        self.node.waited_for_end_of_speaking = False

        return self.node.speech_success, self.node.speech_message

    def set_rgb(self, command="", wait_for_end_of=True):
        
        temp = Int16()
        temp.data = command
        self.node.rgb_mode_publisher.publish(temp)

        self.node.rgb_success = True
        self.node.rgb_message = "Value Sucessfully Sent"

        return self.node.rgb_success, self.node.rgb_message
 
    def wait_for_start_button(self):

        self.node.start_button_state = False

        t = Bool()
        t.data = True
        self.node.flag_start_button_publisher.publish(t)

        while not self.node.start_button_state:
            pass

        t.data = False 
        self.node.flag_start_button_publisher.publish(t)
        
    def get_audio(self, yes_or_no=False, receptionist=False, gpsr=False, restaurant=False, question="", wait_for_end_of=True):

        if yes_or_no or receptionist or gpsr or restaurant:

            # this code continuously asks for new audio info eveytime it gets an error for mishearing
            audio_error_counter = 0
            keywords = "ERROR"
            while keywords=="ERROR":
                
                self.set_speech(filename=question, wait_for_end_of=True)                
                self.set_face("demo7")
                self.node.call_audio_server(yes_or_no=yes_or_no, receptionist=receptionist, gpsr=gpsr, restaurant=restaurant, wait_for_end_of=wait_for_end_of)
                
                if wait_for_end_of:
                    while not self.node.waited_for_end_of_audio:
                        pass
                self.node.waited_for_end_of_audio = False
                self.set_face("demo5")

                keywords = self.node.audio_command  
                
                if keywords=="ERROR":
                    audio_error_counter += 1

                    if audio_error_counter == 2:
                        self.set_speech(filename="generic/please_wait", wait_for_end_of=True)
                        self.calibrate_audio(wait_for_end_of=True)
                        audio_error_counter = 0

                    self.set_speech(filename="generic/not_understand_please_repeat", wait_for_end_of=True)

            return self.node.audio_command  

        else:
            self.node.get_logger().error("ERROR: No audio type selected")
            return "ERROR: No audio type selected" 

    def calibrate_audio(self, wait_for_end_of=True):
            
        self.node.call_calibrate_audio_server(wait_for_end_of=wait_for_end_of)

        if wait_for_end_of:
            while not self.node.waited_for_end_of_calibrate_audio:
                pass
        self.node.waited_for_end_of_calibrate_audio = False

        return self.node.calibrate_audio_success, self.node.calibrate_audio_message 
    
    def set_face(self, command="", custom="", wait_for_end_of=True):
        
        if custom == "":
            temp = String()
            temp.data = command
            self.node.image_to_face_publisher.publish(temp)
        else:
            temp = String()
            temp.data = custom
            self.node.custom_image_to_face_publisher.publish(temp)

        self.node.face_success = True
        self.node.face_message = "Value Sucessfully Sent"

        return self.node.face_success, self.node.face_message
    
    def set_neck(self, position=[0, 0], wait_for_end_of=True):

        self.node.call_neck_position_server(position=position, wait_for_end_of=wait_for_end_of)
        
        if wait_for_end_of:
          while not self.node.waited_for_end_of_neck_pos:
            pass
        self.node.waited_for_end_of_neck_pos = False

        return self.node.neck_success, self.node.neck_message
    
    def set_neck_coords(self, position=[], ang=0.0, wait_for_end_of=True):

        if len(position) == 2:
            self.node.call_neck_coordinates_server(x=position[0], y=position[1], z=0.0, tilt=ang, flag=True, wait_for_end_of=wait_for_end_of)
        elif len(position) == 3:
            print("You tried neck to coordintes using (x,y,z) please switch to (x,y,theta)")
            pass
            # The following line is correct, however since the functionality is not implemented yet, should not be called
            # self.node.call_neck_coordinates_server(x=position[0], y=position[1], z=position[2], tilt=0.0, flag=False, wait_for_end_of=wait_for_end_of)
        else:
            print("Something went wrong")
        
        if wait_for_end_of:
          while not self.node.waited_for_end_of_neck_coords:
            pass
        self.node.waited_for_end_of_neck_coords = False

        return self.node.neck_success, self.node.neck_message
        
    def activate_yolo_pose(self, activate=True, only_detect_person_legs_visible=False, minimum_person_confidence=0.5, minimum_keypoints_to_detect_person=7, only_detect_person_right_in_front=False, only_detect_person_arm_raised=False, characteristics=False, wait_for_end_of=True):
        
        self.node.call_activate_yolo_pose_server(activate=activate, only_detect_person_legs_visible=only_detect_person_legs_visible, minimum_person_confidence=minimum_person_confidence, minimum_keypoints_to_detect_person=minimum_keypoints_to_detect_person, only_detect_person_right_in_front=only_detect_person_right_in_front, only_detect_person_arm_raised=only_detect_person_arm_raised, characteristics=characteristics)

        self.node.activate_yolo_pose_success = True
        self.node.activate_yolo_pose_message = "Activated with selected parameters"

        return self.node.activate_yolo_pose_success, self.node.activate_yolo_pose_message

    def track_person(self, person, body_part="Head", wait_for_end_of=True):

        self.node.call_neck_track_person_server(person=person, body_part=body_part, wait_for_end_of=wait_for_end_of)
        
        if wait_for_end_of:
          while not self.node.waited_for_end_of_track_person:
            pass
        self.node.waited_for_end_of_track_person = False

        return self.node.track_person_success, self.node.track_person_message
 
    
    # main state-machine function
    def main(self):
        # examples of names of states
        # use the names of states rather than just numbers to ease 3rd person code analysis
        Waiting_for_start_button = 0
        Receive_first_guest = 1
        Characteristics_first_guest = 2
        Navigation_to_sofa = 3
        Presentation_host_first_guest = 4
        Receive_second_guest = 5
        Navigation_to_sofa_second = 6
        Presentation_host_first_second_guest = 7
        Final_State = 8

        self.SIDE_TO_LOOK = "Right"

        self.state = Waiting_for_start_button
        self.look_forward = [0, 0]
        self.look_navigation = [0, -40]
        self.look_left = [90, 0]
        self.look_right = [-90, 0]
        self.look_torso = [0, -20]
        self.look_empty_place = [1.0, 2.0]
        # create a look torso
        # create a look sofa

        # debug print
        print("IN NEW MAIN")

        while True:

            if self.state == Waiting_for_start_button:
                print('State 0 = Initial')

    
                self.set_face("demo5")
                self.activate_yolo_pose(activate=False)

                #NECK: LOOKS IN FRONT
                self.set_neck(position=self.look_forward, wait_for_end_of=True)
                self.set_rgb(MAGENTA+ALTERNATE_QUARTERS)
                
                #START TASK
                self.set_speech(filename="receptionist/start_receptionist", wait_for_end_of=True)
                #WAITING START BUTTON
                self.set_speech(filename="generic/waiting_start_button", wait_for_end_of=False)
                self.wait_for_start_button()
                
                # NAVIGATION POSE
                self.set_neck(position=self.look_navigation, wait_for_end_of=True)

                # MOVE TO DOOR LOCALISATION (PLACE TO RECEIVE THE GUEST)
                self.state = Receive_first_guest

            elif self.state == Receive_first_guest:
                print('State 1 = Receive first guest')

                self.calibrate_audio(wait_for_end_of=True)
  
                #NECK: LOOKS IN FRONT 
                self.set_neck(position=self.look_torso, wait_for_end_of=True)

                #RGB: WAITING MODE
                self.set_rgb(YELLOW+ROTATE)
                #SPEAK: I am ready to receive a new guest. Please stand in front of me.
                self.set_speech(filename="receptionist/ready_receive_guest", wait_for_end_of=True)
                #if face = exist:
                # RGB: OK MODE (GREEN)
                # self.set_rgb(GREEN+BLINK_LONG)

                # NECK: LOOKS TO GUEST 1
                self.set_speech(filename="receptionist/presentation_answer_after_green_face", wait_for_end_of=True)

                # AUDIO: RECEIVE NAME AND DRINK OF GUEST
                command = self.get_audio(receptionist=True, question="receptionist/receptionist_question", wait_for_end_of=True)
                print("Finished:", command)
                keyword_list= command.split(" ")
                guest1_name = keyword_list[0] 
                guest1_drink = keyword_list[1]
                print(guest1_name, guest1_drink)
                self.set_speech(filename="receptionist/recep_first_guest_"+guest1_name.lower(), wait_for_end_of=True)
                self.set_speech(filename="receptionist/recep_drink_"+guest1_drink.lower(), wait_for_end_of=True)
                # RGB: OK MODE
                self.set_rgb(GREEN+BLINK_LONG)
                # CAMARA: SAVE IMAGE FROM GUEST 1
            
                self.state = Characteristics_first_guest

            elif self.state == Characteristics_first_guest:
                print('State 2 = Characteristics first guest')
                
                #SAVE IMAGE FROM GUEST WITH NAME: "GUEST1"
                #SPEAK CHARACTERISTICS: AGE, RACE, GENDER, HEIGHT, SHIRT COLOR, PANTS COLORS
                #CALL FUNCTION - PROCESS_CHARACTERISTICS - PROCESSING INFORMATION AS IN 2024 CODE
                #similar like this:
                #get_caract, characteristics, none_variables = analyze_variables(race, age, gender, height, shirt_color, pant_color)


                self.state = Navigation_to_sofa
                
            elif self.state == Navigation_to_sofa:
                print('State 3 = Navigation to sofa')

                #SPEAK:Thank you. Please follow me.
                self.set_speech(filename="receptionist/please_follow_me", wait_for_end_of=True)
                
                self.set_neck(position=self.look_navigation, wait_for_end_of=True)
                if self.SIDE_TO_LOOK == "Right":
                    #MOVE TO SOFA LOCALISATION
                    self.set_neck(position=self.look_right, wait_for_end_of=False)
                    #SPEAK:Please stay on my right until I give you instructions on where to sit.
                    self.set_speech(filename="receptionist/please_stay_on_my_right", wait_for_end_of=True)
                elif self.SIDE_TO_LOOK == "Left":
                    #MOVE TO SOFA LOCALISATION
                    self.set_neck(position=self.look_left, wait_for_end_of=False)
                    #SPEAK:Please stay on my left until I give you instructions on where to sit.
                    self.set_speech(filename="receptionist/please_stay_on_my_left", wait_for_end_of=True)

                self.state = Presentation_host_first_guest

            elif self.state == Presentation_host_first_guest:
                print('State 4 = Presentation host and first guest')
                #NECK: LOOK TO THE SOFA
                #ACTION: FOUND PERSON (HOST)
                #NECK: LOOK TO HOST
                self.set_neck(position=self.look_forward, wait_for_end_of=True)
                #SPEAK:Hello, I will present everyone in this room.
                self.set_speech(filename="receptionist/present_everyone", wait_for_end_of=True)
                #NECK: LOOK TO THE GUEST
                if self.SIDE_TO_LOOK == "Right":
                    #MOVE TO SOFA LOCALISATION
                    self.set_neck(position=self.look_right, wait_for_end_of=False)
                    #SPEAK:Please stay on my right until I give you instructions on where to sit.
                    self.set_speech(filename="receptionist/please_stay_on_my_right", wait_for_end_of=True)
                elif self.SIDE_TO_LOOK == "Left":
                    #MOVE TO SOFA LOCALISATION
                    self.set_neck(position=self.look_left, wait_for_end_of=False)
                    #SPEAK:Please stay on my left until I give you instructions on where to sit.
                    self.set_speech(filename="receptionist/please_stay_on_my_left", wait_for_end_of=True)

                #SPEAK:The host is "NAME" and his favorite drink is "DRINK".
                #NECK: LOOK TO HOST 
                #SPEAK:The first guest name is "NAME" and the favorite drink is "DRINK"
                #ACTION: FOUND AN EMPTY SEAT
                #NECK: LOOK TO AN EMPTY SEAT
                #SPEAK:Please take a sit on the sofa that I'm looking at.
                #NECK: LOOK TO SOFA 
                self.set_neck_coords(position=self.look_empty_place, wait_for_end_of=False)
                self.set_speech(filename="receptionist/please_sit_sofa", wait_for_end_of=True)
                #RGB: OK MODE
                self.set_rgb(GREEN+BLINK_LONG)

                #NECK: LOOKS TO THE FLOOR (NAVIGATION POSE)
                self.set_neck(position=self.look_navigation, wait_for_end_of=True)
                #MOVE TO DOOR LOCALISATION (PLACE TO RECEIVE THE GUEST)
                self.state = Receive_second_guest

            elif self.state == Receive_second_guest:
                print('State 1 = Receive second guest')

                self.calibrate_audio(wait_for_end_of=True)
                #NECK: LOOKS IN FRONT 
                self.set_neck(position=self.look_torso, wait_for_end_of=True)
                #RGB: WAITING MODE
                self.set_rgb(YELLOW+ROTATE)

                self.set_speech(filename="receptionist/ready_receive_guest", wait_for_end_of=True)
                #if face = exist:
                # RGB: OK MODE (GREEN)
                #self.set_rgb(GREEN+BLINK_LONG)
                # NECK: LOOKS TO GUEST 1

                self.set_speech(filename="receptionist/presentation_answer_after_green_face", wait_for_end_of=True)

                # AUDIO: RECEIVE NAME AND DRINK OF GUEST 
                command = self.get_audio(receptionist=True, question="receptionist/receptionist_question", wait_for_end_of=True)
                print("Finished:", command)
                keyword_list= command.split(" ")
                guest2_name = keyword_list[0] 
                guest2_drink = keyword_list[1]
                print(guest2_name, guest2_drink)
                self.set_speech(filename="receptionist/recep_second_guest_"+guest2_name.lower(), wait_for_end_of=True)
                self.set_speech(filename="receptionist/recep_drink_"+guest2_drink.lower(), wait_for_end_of=True)

                self.set_rgb(GREEN+BLINK_LONG)
                self.state = Navigation_to_sofa_second

            elif self.state == Navigation_to_sofa_second:
                print('State 3 = Navigation to sofa')
                #RGB: OK MODE
                #self.set_rgb(GREEN+BLINK_LONG)
                #SPEAK:Thank you. Please follow me.
                self.set_speech(filename="receptionist/please_follow_me", wait_for_end_of=True)
                #NECK: LOOKS TO THE FLOOR (NAVIGATION POSe)
                self.set_neck(position=self.look_navigation, wait_for_end_of=True)
                if self.SIDE_TO_LOOK == "Right":
                    #MOVE TO SOFA LOCALISATION
                    self.set_neck(position=self.look_right, wait_for_end_of=False)
                    #SPEAK:Please stay on my right until I give you instructions on where to sit.
                    self.set_speech(filename="receptionist/please_stay_on_my_right", wait_for_end_of=True)
                elif self.SIDE_TO_LOOK == "Left":
                    #MOVE TO SOFA LOCALISATION
                    self.set_neck(position=self.look_left, wait_for_end_of=False)
                    #SPEAK:Please stay on my left until I give you instructions on where to sit.
                    self.set_speech(filename="receptionist/please_stay_on_my_left", wait_for_end_of=True)

                self.state = Presentation_host_first_second_guest

            elif self.state == Presentation_host_first_second_guest:
                print('State 4 = Presentation host, first and second guest')
                
                #NECK: LOOK TO THE GUEST
                self.set_neck(position=self.look_forward, wait_for_end_of=True)
                
                #SPEAK:Hello, I will present everyone in this room.
                self.set_speech(filename="receptionist/present_everyone", wait_for_end_of=True)
                
                if self.SIDE_TO_LOOK == "Right":
                    #MOVE TO SOFA LOCALISATION
                    self.set_neck(position=self.look_right, wait_for_end_of=False)
                    #SPEAK:Please stay on my right until I give you instructions on where to sit.
                    self.set_speech(filename="receptionist/please_stay_on_my_right", wait_for_end_of=True)
                elif self.SIDE_TO_LOOK == "Left":
                    #MOVE TO SOFA LOCALISATION
                    self.set_neck(position=self.look_left, wait_for_end_of=False)
                    #SPEAK:Please stay on my left until I give you instructions on where to sit.
                    self.set_speech(filename="receptionist/please_stay_on_my_left", wait_for_end_of=True)

                self.set_neck(position=self.look_forward, wait_for_end_of=True)
                self.set_speech(filename="receptionist/dear_host"+"receptionist/dear_guest"+"receptionist/keep_face_clear", wait_for_end_of=True)
                #SPEAK:The host is "NAME" and his favorite drink is "DRINK".
                #SPEAK:Introduce the first guest - name and drink and characteristics
                #NECK: LOOK TO THE SOFA/CHAIRS
                self.set_speech(filename="receptionist/present_everyone", wait_for_end_of=True)
                #ACTION: FOUND PERSONS/LOCALISATIONS
                #ACTION: RECOGNIZE HOST AND GUEST 1 AND ASSOCIATE THEM COORDINATES
                #NECK: LOOK TO HOST

                #SPEAK:Dear host
                self.set_speech(filename="receptionist/dear_host", wait_for_end_of=True)
                #NECK: LOOK TO THE GUEST
                #SPEAK:Dear guest
                self.set_speech(filename="receptionist/dear_guest", wait_for_end_of=True)
                #SPEAK:The second guest name is "NAME" and the favorite drink is "DRINK"
                #ACTION: FOUND AN EMPTY SEAT
                #NECK: LOOK TO AN EMPTY SEAT
                #SPEAK:Please take a sit on the sofa that I'm looking at.
                #NECK: LOOK TO SOFA 
                self.set_neck_coords(position=self.look_empty_place, wait_for_end_of=False)
                self.set_speech(filename="receptionist/please_sit_sofa", wait_for_end_of=True)
                #RGB: OK MODE
                #self.set_rgb(GREEN+BLINK_LONG)
                self.state = Final_State

            elif self.state == Final_State:
                
                print("Finished task!!!")
                #NECK: LOOK IN FRONT
                self.set_neck(position=self.look_forward, wait_for_end_of=True)
                #SPEAK: Thank you. I finished my receptionist task
                self.set_speech(filename="receptionist/finish_receptionist", wait_for_end_of=True)
                #NECK: LOOK TO THE FLOOR
                self.set_neck(position=self.look_navigation, wait_for_end_of=True)
                #self.set_rgb(BLUE+ROTATE)
                # After finishing the task stays in this loop 
                while True:
                    pass

            else:
                pass

    '''def Get_characteristics(race, age, gender, height, shirt_color, pant_color):
        characteristics = []
        none_variables = []

        if race is not None:
            characteristics.append(race)
        else:
            none_variables.append("race")

        if age is not None:
            characteristics.append(age)
        else:
            none_variables.append("age")

        if gender is not None:
            characteristics.append(gender)
        else:
            none_variables.append("gender")

        if height is not None:
            characteristics.append(height)
        else:
            none_variables.append("height")

        if shirt_color is not None:
            characteristics.append(shirt_color)
        else:
            none_variables.append("shirt_color")

        if pant_color is not None:
            characteristics.append(pant_color)
        else:
            none_variables.append("pant_color")

        get_caract = len(characteristics)

        return get_caract, characteristics, none_variables


    def Process_Info(get_caract, characteristics, none_variables):
        if get_caract == 5 or get_caract == 6:
            print("Características:", characteristics)
        elif get_caract == 4 or get_caract == 3 or get_caract == 2 or get_caract == 1 or get_caract == 0:
            print(characteristics)
            if 'age' in none_variables:
                print('age 25 and 32')
            if 'gender' in none_variables:
                print('gender male')
            if 'race' in none_variables:
                print('race caucasian')
            if 'height' in none_variables:
                print('height taller than me')
            if 'shirt_color' in none_variables:
                print('white')
            if 'pant_color' in none_variables:
                pass
            if not none_variables:  # Se não houver variáveis ausentes
                print("nada")



    def main():
        # Variáveis de exemplo para teste
        race = "Caucasian"
        age = 18
        gender = None
        height = None
        shirt_color = "Blue"
        pant_color = None

        # Chamando a função Get_characteristics
        get_caract, characteristics, none_variables = Get_characteristics(race, age, gender, height, shirt_color, pant_color)

        # Mostrando os resultados
        print("Número de características:", get_caract)
        print("Características:", characteristics)
        print("Variáveis None:", none_variables)

        # Processando as informações
        Process_Info(get_caract, characteristics, none_variables)


    if __name__ == "__main__":
        main()'''