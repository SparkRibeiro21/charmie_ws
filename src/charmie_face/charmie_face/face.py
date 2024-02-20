#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from example_interfaces.msg import Bool, String, Float32
from charmie_interfaces.msg import SpeechType, RobotSpeech

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

        self.home = str(Path.home())
        self.midpath = "charmie_ws/src/charmie_face/charmie_face/list_of_temp_faces"
        self.complete_path = self.home+'/'+self.midpath+'/'

        # send initial face
        # self.save_text_file("img", "media/" + file_name + file_extension)
        # self.save_text_file("video", "media/" + "hasb" + ".mp4")
        # print("File saved successfully.")
        # self.save_text_file("img", "media/" + self.complete_path)
        
        new_filename = self.get_filename("temp", ".jpg")
        # the name afther complete path is received from the topic 
        self.copy_file(self.complete_path+"clients_temp.jpg", new_filename)
        self.save_text_file("img", "temp/" + new_filename)
        print("File copied successfully.")


    def save_text_file(self, type, data = ""):
        with open(self.destination + "temp.txt", "w") as file:
            file.write(f"{type}|{data}|")

    def get_filename(self, file_name, extension):
        if not os.path.exists(self.destination + file_name + extension):
            return f"{file_name}{extension}"
        
        count = 1
        while True:
            new_filename = f"{file_name}_{count}{extension}"
            if not os.path.exists(self.destination + new_filename):
                return new_filename
            count += 1

    def copy_file(self, source, file_name):
        shutil.copyfile(source, self.destination + file_name)

    def classify_file(self, file_extension):
        if file_extension.lower() in ['.jpg', '.jpeg', '.png', '.bmp', '.gif']:
            return "img"
        elif file_extension.lower() in ['.mp4', '.avi', '.mkv', '.mov']:
            return "video"
        else:
            return "Unknown"

class FaceNode(Node):
    def __init__(self):
        super().__init__("Face")
        self.get_logger().info("Initialised CHARMIE Face Node")

        self.face = Face()

        ### TOPICS:
        # receive strings to show in face
        self.speech_to_face_subscriber = self.create_subscription(String, "display_speech_face", self.speech_to_face_callback, 10)
        self.image_to_face_subscriber = self.create_subscription(String, "display_image_face", self.image_to_face_callback, 10)
        
    def speech_to_face_callback(self, command: String):

        if command.data != "":
            self.face.save_text_file("text", command.data)
            print("Received Speech String:", command.data)
        else:
            time.sleep(0.5)
            self.face.save_text_file("img", "media/" + "demo5" + ".gif")
            print("Back to Standard face")

    def image_to_face_callback(self, command: String):
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
            self.face.save_text_file("video", "media/" + "hasb.mp4")
            print("Received Image:", command.data)
        else:
            print("Error receiving image, file does not exist")





def main(args=None):
    rclpy.init(args=args)
    node = FaceNode()
    rclpy.spin(node)
    rclpy.shutdown()
