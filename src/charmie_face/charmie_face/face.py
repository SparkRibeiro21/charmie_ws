#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from example_interfaces.msg import String

import time
import os
import shutil
from pathlib import Path


class Face():
    def __init__(self):
        print("New Face Class Initialised")

        # Info to get device host:
        #     1- Go to the device location and open the terminal there
        #     2- write pwd to get the location
        self.destination = "/run/user/1000/gvfs/mtp:host=SAMSUNG_SAMSUNG_Android_52037149ea96c3a1/SanDisk SD card/temp/"

        # info regarding the paths for the custom files intended to be sent to the face
        # by using self.home it automatically adjusts to all computers home file, which may differ since it depends on the username on the PC
        self.home = str(Path.home())
        self.midpath = "charmie_ws/src/charmie_face/charmie_face/list_of_temp_faces"
        self.complete_path = self.home+'/'+self.midpath+'/'

        # saves the latest face received, this is used to return after a speech command ends
        self.last_face_type = ""
        self.last_face_path = ""

    # checks if file exist on the tablet SD card and writes in file so it will appear on face
    def save_text_file(self, type, data = ""):
        with open(self.destination + "temp.txt", "w") as file:
            file.write(f"{type}|{data}|")
        
        # saves the last face to set after finishing speaking
        if type != "text":
            self.last_face_type = type
            self.last_face_path = data

    # checks the filename of custom faces being sent and changes the name so there is no conflict in files with similar names
    def get_filename(self, file_name, extension):
        if not os.path.exists(self.destination + file_name + extension):
            return f"{file_name}{extension}"
        
        count = 1
        while True:
            new_filename = f"{file_name}_{count}{extension}"
            if not os.path.exists(self.destination + new_filename):
                return new_filename
            count += 1

    # copys the custom face file from the /list_of_temp_faces to the tablet SD card
    def copy_file(self, source, file_name):
        shutil.copyfile(source, self.destination + file_name)

    # classifies file between image and video so the the tablet knows which one it is and shows them correctly
    def classify_file(self, file_extension):
        if file_extension.lower() in ['.jpg', '.jpeg', '.png', '.bmp', '.gif']:
            return "img"
        elif file_extension.lower() in ['.mp4', '.avi', '.mkv', '.mov']:
            return "video"
        else:
            return "Unknown"


# ROS2 Face Node
class FaceNode(Node):
    def __init__(self):
        super().__init__("Face")
        self.get_logger().info("Initialised CHARMIE Face Node")

        # Create Face object
        self.face = Face()

        ### ROS2 Parameters ###
        # when declaring a ros2 parameter the second argument of the function is the default value 
        self.declare_parameter("show_speech", True) 
        self.declare_parameter("after_speech_timer", 0.0) 
        self.declare_parameter("initial_face", "demo5") 

        ### Topics (Subscribers) ###   
        # Receive speech strings to show in face
        self.speech_to_face_subscriber = self.create_subscription(String, "display_speech_face", self.speech_to_face_callback, 10)
        # Receive image or video files name to show in face
        self.image_to_face_subscriber = self.create_subscription(String, "display_image_face", self.image_to_face_callback, 10)
        # Receive custom image name to send to tablet and show in face
        self.custom_image_to_face_subscriber = self.create_subscription(String, "display_custom_image_face", self.custom_image_to_face_callback, 10)
      
        # whether or not it is intended to show the speech strings on the face while the robot talks
        self.SHOW_SPEECH = self.get_parameter("show_speech").value
        # the time after every speaked sentence, that the face remains the speech after finished the speakers (float) 
        self.AFTER_SPEECH_TIMER = self.get_parameter("after_speech_timer").value
        # which face should be displayed after initialising the face node (string) 
        self.INITIAL_FACE = self.get_parameter("initial_face").value
        
        self.get_logger().info("Initial Face Received is: %s" %self.INITIAL_FACE)

        # sends initial face
        first_face = String()
        first_face.data = self.INITIAL_FACE
        self.image_to_face_callback(first_face)

        
    # Receive speech strings to show in face  
    def speech_to_face_callback(self, command: String):

        if self.SHOW_SPEECH:
            if command.data != "":
                self.face.save_text_file("text", command.data)
                print("Received Speech String:", command.data)
            else:
                time.sleep(self.AFTER_SPEECH_TIMER)
                # after receiving the end of speech command, it sends to the face the latest face sent before the speech command
                self.face.save_text_file(self.face.last_face_type, self.face.last_face_path)
                print(self.face.last_face_type, self.face.last_face_path)
                print("Back to Standard face")


    # Receive image or video files name to show in face
    def image_to_face_callback(self, command: String):

        # list of all faces available, must be edited for every new face available
        if command.data == "demo1":
            self.face.save_text_file("img", "media/" + "demo1.png")
            print("Received Image:", command.data)
        elif command.data == "demo2":
            self.face.save_text_file("img", "media/" + "demo2.jpg")
            print("Received Image:", command.data)
        elif command.data == "demo3":
            self.face.save_text_file("img", "media/" + "demo3.png")
            print("Received Image:", command.data)
        elif command.data == "demo4":
            self.face.save_text_file("img", "media/" + "demo4.jpg")
            print("Received Image:", command.data)
        elif command.data == "demo5":
            self.face.save_text_file("img", "media/" + "demo5.gif")
            print("Received Image:", command.data)
        elif command.data == "demo6":
            self.face.save_text_file("img", "media/" + "demo6.gif")
            print("Received Image:", command.data)
        elif command.data == "demo7":
            self.face.save_text_file("img", "media/" + "demo7.gif")
            print("Received Image:", command.data)
        elif command.data == "demo8":
            self.face.save_text_file("video", "media/" + "demo8.MOV")
            print("Received Image:", command.data)
        elif command.data == "demo9":
            self.face.save_text_file("img", "media/" + "demo9.jpg")
            print("Received Image:", command.data)

        elif command.data == "help_pick_cup":
            self.face.save_text_file("img", "media/" + "help_pick_cup.jpg")
        elif command.data == "help_pick_bowl":
            self.face.save_text_file("img", "media/" + "help_pick_bowl.jpg")
        elif command.data == "help_pick_milk":
            self.face.save_text_file("img", "media/" + "help_pick_milk.jpg")
        elif command.data == "help_pick_cereal":
            self.face.save_text_file("img", "media/" + "help_pick_cereal.jpg")
        elif command.data == "help_pick_spoon":
            self.face.save_text_file("img", "media/" + "help_pick_spoon.jpg")

        else:
            print("Error receiving image, file does not exist")


    # Receive custom image name to send to tablet and show in face
    def custom_image_to_face_callback(self, command: String):

        # checks whether file exists, maybe there was some typo 
        isExisting = os.path.exists(self.face.complete_path + command.data + ".jpg")
        
        if isExisting:
            # checks the filename of custom faces being sent and changes the name so there is no conflict in files with similar names
            new_filename = self.face.get_filename(command.data, ".jpg")
            
            # the name afther complete path is received from the topic 
            # self.face.copy_file(self.face.complete_path+"clients_temp.jpg", new_filename)
            self.face.copy_file(self.face.complete_path + command.data + ".jpg", new_filename)
            
            # checks if file exist on the tablet SD card and writes in file so it will appear on face
            self.face.save_text_file("img", "temp/" + new_filename)
            print("File copied successfully.")
        else:
            print("Error! File not found!")

def main(args=None):
    rclpy.init(args=args)
    node = FaceNode()
    rclpy.spin(node)
    rclpy.shutdown()
