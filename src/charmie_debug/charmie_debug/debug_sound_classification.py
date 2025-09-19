#!/usr/bin/env python3
import rclpy
import threading
import time
from charmie_std_functions.task_ros2_and_std_functions import ROS2TaskNode, RobotStdFunctions
import random

# Constant Variables to ease RGB_MODE coding
RED, GREEN, BLUE, YELLOW, MAGENTA, CYAN, WHITE, ORANGE, PINK, BROWN  = 0, 10, 20, 30, 40, 50, 60, 70, 80, 90
SET_COLOUR, BLINK_LONG, BLINK_QUICK, ROTATE, BREATH, ALTERNATE_QUARTERS, HALF_ROTATE, MOON, BACK_AND_FORTH_4, BACK_AND_FORTH_8  = 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
CLEAR, RAINBOW_ROT, RAINBOW_ALL, POLICE, MOON_2_COLOUR, PORTUGAL_FLAG, FRANCE_FLAG, NETHERLANDS_FLAG = 255, 100, 101, 102, 103, 104, 105, 106

ros2_modules = {
    "charmie_arm":                  False,
    "charmie_audio":                False,
    "charmie_face":                 False,
    "charmie_head_camera":          False,
    "charmie_hand_camera":          False,
    "charmie_base_camera":          False,
    "charmie_gamepad":              False,
    "charmie_lidar":                False,
    "charmie_lidar_bottom":         False,
    "charmie_lidar_livox":          False,
    "charmie_llm":                  False,
    "charmie_localisation":         False,
    "charmie_low_level":            False,
    "charmie_navigation":           False,
    "charmie_nav2":                 False,
    "charmie_neck":                 False,
    "charmie_radar":                False,
    "charmie_sound_classification": True,
    "charmie_speakers":             True,
    "charmie_tracking":             False,
    "charmie_yolo_objects":         False,
    "charmie_yolo_pose":            False,
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

    # main state-machine function
    def main(self):
        # Waiting_for_start_button = 0
        Sound_classification = 1
        Continuous_sound_classification_wfeo_true = 2
        Continuous_sound_classification_wfeo_false = 3
        Final_State = 4

        # VARS ...
        self.state = Continuous_sound_classification_wfeo_true
    
        self.robot.set_face("charmie_face")
        print("IN NEW MAIN")
        
        while True:

            if self.state == Sound_classification:
                print('State 6 = Sound Classification')

                labels, scores = self.robot.get_sound_classification(question="sound_classification/sound_classification_start_"+str(random.randint(1, 6)), duration=3.0, score_threshold=0.1, wait_for_end_of=True)
                
                print("Heard Sounds:")
                print("Scores:\tLabels:")
                for l, s in zip(labels, scores):
                    print(f"{s:.2f} - {l}")
                print("\n")

                time.sleep(5)

            if self.state == Continuous_sound_classification_wfeo_true:
                ### CONTINUOUS SOUND CLASSIFICATION EXAMPLE - WAIT FOR END OF = TRUE

                s, m, label, score = self.robot.get_continuous_sound_classification(break_sounds=["finger snapping", "whistling", 'whistle'], timeout=10, score_threshold=0.1, wait_for_end_of=True)
                print(s, m, label, score)
                if m.lower() == "timeout":
                    print("TIMEOUT REACHED")

                # next state
                self.state = Final_State

            if self.state == Continuous_sound_classification_wfeo_false:
                ### CONTINUOUS SOUND CLASSIFICATION EXAMPLE - WAIT FOR END OF = FALSE
                
                self.robot.get_continuous_sound_classification(break_sounds=["finger snapping", "whistling", 'whistle'], timeout=20, score_threshold=0.1, wait_for_end_of=False)
                message_received = False
                while not message_received:
                    message_received, s, m, label, score = self.robot.is_get_continuous_sound_classification_done()
                    print(message_received, s, m, label, score)
                    time.sleep(0.5)
                
                # next state
                self.state = Final_State
            
            elif self.state == Final_State:
                # self.node.speech_str.command = "I have finished my restaurant task." 
                # self.node.speaker_publisher.publish(self.node.speech_str)
                # self.wait_for_end_of_speaking()  
                self.state += 1
                print("Finished task!!!")

            else:
                pass