#!/usr/bin/env python3
from rclpy.node import Node
import rclpy
import time
from charmie_interfaces.msg import ListOfStrings
from charmie_interfaces.srv import GetLLMDemo, GetLLMGPSR, GetLLMConfirmCommand

import json

##### NOTAS: #####
# temos de adicioanr infos sobre:
# - que dia, mês, ano, dia da semana em que estamos (pode sair no GPSR)
# - Que horas são (pode sair no GPSR)
# - Onde é que o robô está, local, cidade e país, mas também o evento e o charmie saber dizer o que é o evento (RoboParty, RoboCup)

from charmie_llm.llm_demo_description import LLM_demo_description
from charmie_llm.llm_planner_desciption import LLM_planner_description
from charmie_llm.llm_info_extraction_description import LLM_info_extraction_description

# main function that already creates the thread for the task state machine
def main(args=None):
    rclpy.init(args=args)
    node = LLMNode()
    rclpy.spin(node)
    rclpy.shutdown()

class LLMNode(Node):

    def __init__(self):
        # create a robot instance so use all standard CHARMIE functions
        super().__init__("LLM")
        self.get_logger().info("Initialised CHARMIE LLM Node")

        # LLM objects declaration must come before the service declaration, so that if there is any error, in GUI never shows that LLM was initialized 
        self.llm_demo_description = LLM_demo_description() 
        self.llm_planner_description = LLM_planner_description()
        self.llm_info_extraction_description = LLM_info_extraction_description()

        self.llm_demonstration_server = self.create_service(GetLLMDemo, "llm_demonstration", self.llm_demonstration_callback)
        self.llm_confirm_command_server = self.create_service(GetLLMConfirmCommand, "llm_confirm_command", self.llm_confirm_command_callback)
        self.llm_gpsr_server = self.create_service(GetLLMGPSR, "llm_gpsr", self.llm_gpsr_callback)

    def llm_demonstration_callback(self, request, response):
        # Type of service received: 
        # 
        # string command        
        # ---
        # string answer # LLM Response to the question asked to the robot 
        
        self.get_logger().info("LLM DEMO REQUEST RECEIVED")
        print("Received:", request.command)

        # FOR THE DEMO
        # response.answer = self.llm_demo_description.run(request.command) 

        # TEMPORARY IN HERE!!! FOR THE INFO EXTRACTION
        request_command = request.command

        info_type, command = request_command.split(" - ", 1)

        extracted_info = self.llm_info_extraction_description.extract_info(request=command, info_type=info_type)
    
        print("Extracted favorite drink:", extracted_info)

        response.answer = extracted_info

        return response
    


    def llm_confirm_command_callback(self, request, response):
        # Type of service received: 
        # 
        # string command        
        # ---
        # string answer # LLM Response to the question asked to the robot 

        self.get_logger().info("LLM CONFIRM COMMAND REQUEST RECEIVED")
        print("Received:", request.command)
        
        time.sleep(3.0)

        example = "this is a test"
        response.answer = example

        return response    

    def llm_gpsr_callback(self, request, response): # this only exists to have a service where we can: "while not self.arm_trigger_client.wait_for_service(1.0):"
        # Type of service received: 
        # 
        # string command
        # ---
        # ListOfStrings answer # List of sub tasks divisions the robot must perform to complete a GPSR task
        
        self.get_logger().info("LLM GPSR REQUEST RECEIVED")
        print("Received:", request.command)

        # sends the command to the LLM and gets the response (the generated plan)
        llm_response = self.llm_planner_description.handle_request(request.command)
        
        # print (f"LLM Output:", llm_response)
        # print (f"LLM response.output_text:", llm_response.output)
        
        example = ListOfStrings()  # list of the subtasks in a structured format for the robot to execute (on the left of the '-' is the type of task and on the right the parameters)

        for item in llm_response.output:
            if item.type == "function_call":

                args = json.loads(item.arguments)

                if item.name == "move_to_room":

                    example.strings.append(f"MoveToRoom-{args['room']}")
                
                elif item.name == "move_to_furniture":
                    
                    example.strings.append(f"MoveToFurniture-{args['furniture']}")
                
                elif item.name == "move_to_person_through_name":
                    
                    example.strings.append(f"MoveToPersonName-{args['person_name']}")
                
                elif item.name == "move_to_person_with_pose":
                    
                    example.strings.append(f"MoveToPersonPose-{args['person_pose']}")
                
                elif item.name == "move_to_person_with_clothing":

                    example.strings.append(f"MoveToPersonClothing-{args['person_clothing']}")

                elif item.name == "move_to_initial_person_position":
                    
                    example.strings.append(f"MoveToInitialPersonPosition")
                
                elif item.name == "pick_object":

                    example.strings.append(f"Pick-{args['name_of_object']}")

                elif item.name == "place_object_on_furniture":

                    example.strings.append(f"Place-{args['name_of_object']},{args['furniture']}")
                elif item.name == "hand_object_to_person":

                    example.strings.append(f"HandObject")
                
        response.answer = example  
        return response

