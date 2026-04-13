#!/usr/bin/env python3

from rclpy.node import Node
import rclpy
import time
from charmie_interfaces.msg import ListOfStrings
from charmie_interfaces.srv import GetLLMResponse

import json

##### NOTAS: #####
# temos de adicioanr infos sobre:
# - que dia, mês, ano, dia da semana em que estamos (pode sair no GPSR)
# - Que horas são (pode sair no GPSR)
# - Onde é que o robô está, local, cidade e país, mas também o evento e o charmie saber dizer o que é o evento (RoboParty, RoboCup)


### Classes that handle the ollama LLMs 
from charmie_llm.llm_planner_desciption import Ollama_planner_description
from charmie_llm.llm_info_extraction_description import Ollama_info_extraction_description


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
        self.get_logger().info("Initialised CHARMIE LLM Offline Ollama Node")

        # LLM objects declaration must come before the service declaration, so that if there is any error, in GUI never shows that LLM was initialized 
        # self.llm_demo_description = LLM_demo_description() 
        # self.llm_planner_description = LLM_planner_description()
        # self.llm_info_extraction_description = LLM_info_extraction_description()

        #self.ollama_planner = Ollama_planner_description()
        self.ollama_info_extraction = Ollama_info_extraction_description()

        ### HERE WE NEED TO ADD A DUMMY FUNCTION TO GET A FIRST RESPONSE FROM THE LLM
        ### THIS IS BECAUSE THE FIRST TIME WE CALL THE LLM IT TAKES A LONG TIME TO LOAD, SO WE WANT TO DO IT IN THE BEGINNING, SO THAT WHEN WE CALL IT LATER ON, IT IS ALREADY LOADED AND READY TO RESPOND QUICKLY
        ### THIS NEEDS TO HAPPEN BEFORE THE SERVERS CREATION, SO THAT WHEN OTHER NODES CHECK THE SERVICE AVAILABILITY, IT IS ALREADY AVAILABLE AND NOT WAITING FOR THE LLM TO LOAD

        self.llm_ollama_demonstration_server    = self.create_service(GetLLMResponse, "llm_ollama_demonstration",   self.llm_ollama_demonstration_callback)
        self.llm_ollama_information_server      = self.create_service(GetLLMResponse, "llm_ollama_information",     self.llm_ollama_information_callback)
        self.llm_ollama_gpsr_high_level_server  = self.create_service(GetLLMResponse, "llm_ollama_gpsr_high_level", self.llm_ollama_gpsr_high_level_callback)
        self.llm_ollama_gpsr_low_level_server   = self.create_service(GetLLMResponse, "llm_ollama_gpsr_low_level",  self.llm_ollama_gpsr_low_level_callback)
        self.get_logger().info("Initialised CHARMIE LLM Offline Ollama Response Servers")

    def llm_ollama_demonstration_callback(self, request, response):
        # Type of service received: 
        # string command          # The command to be sent to the LLM
        # string mode              # defines the operation mode for the LLM
        # ---
        # ListOfStrings answer    # List of sub tasks divisions the robot must perform to complete a task
        
        self.get_logger().info("LLM DEMO REQUEST RECEIVED")
        print("Received:", request.command)

        """ # FOR THE DEMO
        # response.answer = self.llm_demo_description.run(request.command) 

        # TEMPORARY IN HERE!!! FOR THE INFO EXTRACTION
        request_command = request.command

        info_type, command = request_command.split(" - ", 1)

        extracted_info = self.llm_info_extraction_description.extract_info(request=command, info_type=info_type)
    
        print("Extracted favorite drink:", extracted_info)
 
        response.answer = extracted_info"""
        
        los = ListOfStrings()
        los.strings.append("This is 1 demonstration response from the LLM.")
        los.strings.append("This is 2 demonstration response from the LLM.")
        los.strings.append("This is 3 demonstration response from the LLM.")
        
        response.answer = los

        return response
    

    def llm_ollama_information_callback(self, request, response):
        # Type of service received: 
        # string command          # The command to be sent to the LLM
        # string mode              # defines the operation mode for the LLM
        # ---
        # ListOfStrings answer    # List of sub tasks divisions the robot must perform to complete a task
        
        self.get_logger().info("LLM INFORMATION REQUEST RECEIVED")
        print("Received:", request.command)

        """ # FOR THE DEMO
        # response.answer = self.llm_demo_description.run(request.command) 


        # TEMPORARY IN HERE!!! FOR THE INFO EXTRACTION
        request_command = request.command

        info_type, command = request_command.split(" - ", 1)

        extracted_info = self.llm_info_extraction_description.extract_info(request=command, info_type=info_type)
    
        print("Extracted favorite drink:", extracted_info)

        response.answer = extracted_info """

        request_command = request.command
        info_type = request.mode

        extracted_info = self.ollama_info_extraction.extract_info(request=request_command, info_type=info_type)
        print("Extracted " + info_type + ":", extracted_info)


        los = ListOfStrings()
        los.strings.append(extracted_info)
        
        response.answer = los

        return response
    

    def llm_ollama_gpsr_high_level_callback(self, request, response):
        # Type of service received: 
        # string command          # The command to be sent to the LLM
        # string mode              # defines the operation mode for the LLM
        # ---
        # ListOfStrings answer    # List of sub tasks divisions the robot must perform to complete a task
        
        self.get_logger().info("LLM GPSR HIGH LEVEL REQUEST RECEIVED")
        print("Received:", request.command)

        """ # sends the command to the LLM and gets the response (the generated plan)
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
                
        response.answer = example   """

        los = ListOfStrings()
        los.strings.append("This is 7 demonstration response from the LLM.")
        los.strings.append("This is 8 demonstration response from the LLM.")
        los.strings.append("This is 9 demonstration response from the LLM.")
        
        response.answer = los

        return response


    def llm_ollama_gpsr_low_level_callback(self, request, response):
        # Type of service received: 
        # string command          # The command to be sent to the LLM
        # string mode              # defines the operation mode for the LLM
        # ---
        # ListOfStrings answer    # List of sub tasks divisions the robot must perform to complete a task
        
        self.get_logger().info("LLM GPSR LOW LEVEL REQUEST RECEIVED")
        print("Received:", request.command)

        """ # sends the command to the LLM and gets the response (the generated plan)
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
                
        response.answer = example   """

        los = ListOfStrings()
        los.strings.append("This is 10 demonstration response from the LLM.")
        los.strings.append("This is 11 demonstration response from the LLM.")
        los.strings.append("This is 12 demonstration response from the LLM.")
        
        response.answer = los

        return response

