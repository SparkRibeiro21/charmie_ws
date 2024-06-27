#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from example_interfaces.msg import String
from charmie_interfaces.srv import SetFace

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
        # print("DATA:", data)

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

        self.face = Face()

        ### ROS2 Parameters ###
        # when declaring a ros2 parameter the second argument of the function is the default value 
        self.declare_parameter("show_speech", True) 
        self.declare_parameter("after_speech_timer", 0.0) 
        self.declare_parameter("initial_face", "charmie_face") 

        

        ### Topics (Subscribers) ###   
        # Receive speech strings to show in face
        self.speech_to_face_subscriber = self.create_subscription(String, "display_speech_face", self.speech_to_face_callback, 10)
        
        ### Services (Server) ###   
        self.server_face_command = self.create_service(SetFace, "face_command", self.callback_face_command) 
        self.get_logger().info("Face Servers have been started")

        # whether or not it is intended to show the speech strings on the face while the robot talks
        self.SHOW_SPEECH = self.get_parameter("show_speech").value
        # the time after every speaked sentence, that the face remains the speech after finished the speakers (float) 
        self.AFTER_SPEECH_TIMER = self.get_parameter("after_speech_timer").value
        # which face should be displayed after initialising the face node (string) 
        self.INITIAL_FACE = self.get_parameter("initial_face").value
        
        self.get_logger().info("Initial Face Received is: %s" %self.INITIAL_FACE)

        # sends initial face
        self.image_to_face(self.INITIAL_FACE)

        # easier debug when testing custom faces 
        # self.image_to_face("charmie_face_green")
    

    # Callback for all face commands received
    def callback_face_command(self, request, response):
        print("Received request", request.command)
 
        # Type of service received: 
        # string command # type of face that is commonly used and is always the same, already in face (tablet) SD card (i.e. hearing face and standard blinking eyes face)
        # string custom # type of face that is custom, not previously in face (tablet) SD card (i.e. show detected person or object in the moment)
        # ---
        # bool success   # indicate successful run of triggered service
        # string message # informational, e.g. for error messages.

        if request.command != "":
            response.success, response.message = self.image_to_face(command=request.command)
        elif request.custom != "":
            response.success, response.message = self.custom_image_to_face(command=request.custom)
        else:
            response.success = False
            response.message = "No standard or custom face received."

        return response

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
                # print("Back to last face:", self.face.last_face_path)
                
    # Receive image or video files name to show in face
    def image_to_face(self, command):
        
        # since the extension is not known, a system to check all filenames disregarding the extension had to be created
        file_exists = False
        files = os.listdir(self.face.destination_media)
        for file in files:
            file_name, file_extension = os.path.splitext(file)
            if file_name == command:
                file_exists = True

        if file_exists:
            self.get_logger().info("FACE received (standard) - %s" %command)
            self.face.save_text_file("media/" + command)
            return True, "Face received (standard) sucessfully displayed"

        else:
            self.get_logger().error("FACE received (standard) does not exist! - %s" %command)
            return False, "FACE received (standard) does not exist."
    
    # Receive custom image name to send to tablet and show in face
    def custom_image_to_face(self, command):

        # checks whether file exists, maybe there was some typo 
        file_exists = os.path.exists(self.face.complete_path + command + ".jpg")
        
        if file_exists:
            self.get_logger().info("FACE received (custom) - %s" %command)
            
            # checks the filename of custom faces being sent and changes the name so there is no conflict in files with similar names
            new_filename = self.face.get_filename(command, ".jpg")
            
            # the name afther complete path is received from the topic 
            self.face.copy_file(self.face.complete_path + command + ".jpg", new_filename + ".jpg")
            
            # checks if file exist on the tablet SD card and writes in file so it will appear on face
            self.face.save_text_file("temp/" + new_filename)
            # print("File copied successfully.")
            return True, "Face received (custom) sucessfully displayed"

        else:
            self.get_logger().error("FACE received (custom) does not exist! - %s" %command)
            return False, "FACE received (custom) does not exist."


def main(args=None):
    rclpy.init(args=args)
    node = FaceNode()
    rclpy.spin(node)
    rclpy.shutdown()
