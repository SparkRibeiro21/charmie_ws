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
        # print("New Face Class Initialised")

        # Info to get device host:
        #     1- Go to the device location and open the terminal there
        #     2- write pwd to get the location
        self.destination = "/run/user/1000/gvfs/mtp:host=SAMSUNG_SAMSUNG_Android_52037149ea96c3a1/SanDisk SD card/"
        self.destination_temp = self.destination + "temp/"
        self.destination_media = self.destination + "media/"

        # info regarding the paths for the custom files intended to be sent to the face
        # by using self.home it automatically adjusts to all computers home file, which may differ since it depends on the username on the PC
        self.home = str(Path.home())
        self.midpath = "charmie_ws/src/charmie_face/charmie_face/list_of_temp_faces"
        self.complete_path = self.home+'/'+self.midpath+'/'

        # saves the latest face received, this is used to show the face used pre speaking, after finishing speaking 
        self.last_face_path = ""

    # checks if file exist on the tablet SD card and writes in file so it will appear on face
    def save_text_file(self, data = ""):
        print("DATA:", data)

        with open(self.destination_temp + "temp.txt", "w") as file:
            file.write(f"{data}")
        
        # saves the last face to set after finishing speaking
        if data.startswith("media/") or data.startswith("temp/"):
            self.last_face_path = data

    # checks the filename of custom faces being sent and changes the name so there is no conflict in files with similar names
    def get_filename(self, file_name, extension):
        if not os.path.exists(self.destination_temp + file_name + extension):
            return f"{file_name}"
        
        count = 1
        while True:
            new_filename = f"{file_name}_{count}{extension}"
            new_filename_without_extension = f"{file_name}_{count}"
            if not os.path.exists(self.destination_temp + new_filename):
                # print(new_filename_without_extension)
                return new_filename_without_extension
            count += 1

    # copys the custom face file from the /list_of_temp_faces to the tablet SD card
    def copy_file(self, source, file_name):
        shutil.copyfile(source, self.destination_temp + file_name)


# ROS2 Face Node
class FaceNode(Node):
    def __init__(self):
        super().__init__("Face")
        self.get_logger().info("Initialised CHARMIE FACE Node")

        # Create Face object
        self.face = Face()

        ### ROS2 Parameters ###
        # when declaring a ros2 parameter the second argument of the function is the default value 
        self.declare_parameter("show_speech", True) 
        self.declare_parameter("after_speech_timer", 0.0) 
        self.declare_parameter("initial_face", "demo1") 

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
                self.face.save_text_file(command.data)
                self.get_logger().info("FACE received (text) - %s" %command.data)
                # print("Received Speech String:", command.data)
            else:
                time.sleep(self.AFTER_SPEECH_TIMER)
                # after receiving the end of speech command, it sends to the face the latest face sent before the speech command
                self.face.save_text_file(self.face.last_face_path)
                # print(self.face.last_face_path)
                # print("Back to Standard face")


    # Receive image or video files name to show in face
    def image_to_face_callback(self, command: String):
        
        ##### MISSING ERROR MESSAGE WHEN FILE DOES NOT EXIST, BUT NOW I HAVE TO DO THAT WITHOUT KNOWING THE EXTENSION
        self.get_logger().info("FACE received (standard) - %s" %command.data)
        self.face.save_text_file("media/" + command.data)


    # Receive custom image name to send to tablet and show in face
    def custom_image_to_face_callback(self, command: String):

        # checks whether file exists, maybe there was some typo 
        isExisting = os.path.exists(self.face.complete_path + command.data + ".jpg")
        
        if isExisting:
            self.get_logger().info("FACE received (custom) - %s" %command.data)
            
            # checks the filename of custom faces being sent and changes the name so there is no conflict in files with similar names
            new_filename = self.face.get_filename(command.data, ".jpg")
            
            # the name afther complete path is received from the topic 
            # self.face.copy_file(self.face.complete_path+"clients_temp.jpg", new_filename)
            self.face.copy_file(self.face.complete_path + command.data + ".jpg", new_filename + ".jpg")
            
            # checks if file exist on the tablet SD card and writes in file so it will appear on face
            self.face.save_text_file("temp/" + new_filename)
            # print("File copied successfully.")

        else:
            self.get_logger().error("FACE received (custom) does not exist! - %s" %command.data)


def main(args=None):
    rclpy.init(args=args)
    node = FaceNode()
    rclpy.spin(node)
    rclpy.shutdown()
