#!/usr/bin/env python3
from rclpy.node import Node
import rclpy
import time
from charmie_interfaces.msg import ListOfStrings
from charmie_interfaces.srv import GetLLMDemo, GetLLMGPSR, GetLLMConfirmCommand

##### NOTAS: #####
# temos de adicioanr infos sobre:
# - que dia, mês, ano, dia da semana em que estamos (pode sair no GPSR)
# - Que horas são (pode sair no GPSR)
# - Onde é que o robô está, local, cidade e país, mas também o evento e o charmie saber dizer o que é o evento (RoboParty, RoboCup)

from charmie_llm.llm_demo_description import LLM_demo_description

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
        self.llm_demo_description = LLM_demo_description() # TODO: add gpsr and question_debug

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

        response.answer = self.llm_demo_description.run(request.command) 

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
        
        time.sleep(3.0)
        
        ### YOUR CODE HERE
        example = ListOfStrings()
        example.strings.append("Navigation-KitchenTable")
        example.strings.append("SearchForObject-Milk")
        example.strings.append("ArmPick-Milk")
        example.strings.append("Navigation-SideTable")
        example.strings.append("ArmPlace-SideTable")
        example.strings.append("Navigation-LivingRoom")
        example.strings.append("Speak-/gpsr/arrived_living_room")
        response.answer = example
        
        return response    
