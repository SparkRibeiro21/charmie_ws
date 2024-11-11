#!/usr/bin/env python3
from rclpy.node import Node
import rclpy
# import threading
import time
from charmie_interfaces.msg import ListOfStrings
from charmie_interfaces.srv import GetLLMDemo, GetLLMGPSR

##### NOTAS: #####
# temos de adicioanr infos sobre:
# - que dia, mês, ano, dia da semana em que estamos (pode sair no GPSR)
# - Que horas são (pode sair no GPSR)
# - Onde é que o robô está, local, cidade e país, mas também o evento e o charmie saber dizer o que é o evento (RoboParty, RoboCup)

from llm_demo_description import LLM_Demo_description

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
        self.llm_demo_description =LLM_Demo_description () # TODO: add gpsr and question_debug
        self.llm_demonstration_server = self.create_service(GetLLMDemo, "llm_demonstration", self.llm_demonstration_callback)
        self.llm_gpsr_server = self.create_service(GetLLMGPSR, "llm_gpsr", self.llm_gpsr_callback)


    def llm_demonstration_callback(self, request, response): # this only exists to have a service where we can: "while not self.arm_trigger_client.wait_for_service(1.0):"
        # Type of service received: 
        # 
        # string command        
        # ---
        # string answer # LLM Response to the question asked to the robot 
        
        self.get_logger().info("LLM DEMO REQUEST RECEIVED")
        print("Received:", request.command)
        response.answer = self.llm_demo.run(request.command) 
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


        ##### Slender Main #####
        """
        # States in LLM Task
        self.Waiting_for_task_start = 0
        self.Approach_kitchen_counter = 1
        self.Picking_up_spoon = 2
        self.Picking_up_milk = 3
        self.Picking_up_cereal = 4
        self.Picking_up_bowl = 5
        self.Approach_kitchen_table = 6
        self.Placing_bowl = 7
        self.Placing_cereal = 8
        self.Placing_milk = 9
        self.Placing_spoon = 10
        self.Final_State = 11
        
        # State the robot starts at, when testing it may help to change to the state it is intended to be tested
        self.state = self.Waiting_for_task_start

        # debug print to know we are on the main start of the task
        self.robot.node.get_logger().info("In LLM Main...")

        client = OpenAI(api_key=api_key)
        
        # Create a thread for the conversation
        
        #thread = client.beta.threads.create()
        

        # Create a vector store caled "Financial Statements"
        vector_store = client.beta.vector_stores.create(name="Financial Statements")
        
        # Ready the files for upload to OpenAI
        folder_path="./src/charmie_llm/charmie_llm/Files/"
        file_paths = ["tdp2023.pdf", "tdp2024.pdf","Actual Config.txt","Actual Team Members.txt", "Assistant Tasks.txt","Hardware Overview.txt"]  # List of your PDF files
        #file_streams = [open(folder_path+path, "rb") for path in file_paths]

        # Use the upload and poll SDK helper to upload the files, add them to the vector store,
        # and poll the status of the file batch for completion.
        #file_batch = client.beta.vector_stores.file_batches.upload_and_poll(
        #vector_store_id=vector_store.id, files=file_streams
        #)


        thread_id= "thread_M5Ot2oA7DdtK2moSe3cglsLz"
        assistant_id= "asst_6DJDcqFuFp1RtpSx4TFvsYsi"
        # Create the assistant
        '''assistant = client.beta.assistants.create(
            name="Charmie",
            instructions=instructions_text,
            model="gpt-3.5-turbo-0125",
            tools=math_tools,
            tool_resources={"file_search": {"vector_store_ids": [vector_store.id]}},
        )'''


        ## File Upload (TDP, Articles, ....)

        # Start the conversation loop
        max_turns = 5
        assistant_res="Hi! I am Charmie. How can I help you?"
        while True:
            #print("State:", self.state, "- Just chatting")
            #self.set_speech(filename="generic/waiting_door_open", wait_for_end_of=True)
            self.robot.node.get_logger().info("Charmie:"+assistant_res)
            self.robot.calibrate_audio(wait_for_end_of=True)
            # Get user input
            
            #print("Assistant: ",assistant_res)
            command = self.robot.get_audio(gpsr=True,question=assistant_res)
            

            #(f"\nSay something: {command}")
            #print("Human: ",command)
            # Send user input as a message in the thread
            user_message = client.beta.threads.messages.create(
                thread_id=thread_id,
                role="user",
                content=command
            )

            # Wait for the assistant's response
            run = client.beta.threads.runs.create_and_poll(
                thread_id=thread_id,
                assistant_id=assistant_id
            )
            #print("Thread: ", thread_id)
            #print("Assist: ", assistant_id)
            # The agent loop. `max_turns` will set a limit on the number of LLM calls made inside the agent loop.
            # Its better to set a limit since LLM calls are costly.
            for turn in range(max_turns):

                # Fetch the last message from the thread
                messages = client.beta.threads.messages.list(
                    thread_id=thread_id,
                    run_id=run.id,
                    order="desc",
                    limit=1,
                )

                # Check for the terminal state of the Run.
                # If state is "completed", exit agent loop and return the LLM response.
                if run.status == "completed":
                    assistant_res: str = next(
                        (
                            content.text.value
                            for content in messages.data[0].content
                            if content.type == "text"
                        ),
                        None,
                    )

                #print(assistant_res)#return assistant_res
                # If state is "requires_action", function calls are required. Execute the functions and send their outputs to the LLM.
                if run.status == "requires_action":
                    func_tool_outputs = []

                    # LLM can ask for multiple functions to be executed. Execute all function calls in loop and
                    # append the results into `func_tool_outputs` list.
                    for tool in run.required_action.submit_tool_outputs.tool_calls:
                        # parse the arguments required for the function call from the LLM response
                        args = (
                            json.loads(tool.function.arguments)
                            if tool.function.arguments
                            else {}
                        )
                        func_output = tool_callables[tool.function.name](**args)

                        # OpenAI needs the output of the function call against the tool_call_id
                        func_tool_outputs.append(
                            {"tool_call_id": tool.id, "output": str(func_output)}
                        )

                    # Submit the function call outputs back to OpenAI
                    run = client.beta.threads.runs.submit_tool_outputs_and_poll(
                        thread_id=thread_id, run_id=run.id, tool_outputs=func_tool_outputs
                    )

                    # Continue the agent loop.
                    # Agent will check the output of the function output submission as part of next iteration.
                    continue

                # Handle errors if terminal state is "failed"
                else:
                    if run.status == "failed":
                        log.error(
                            f"OpenAIFunctionAgent turn-{turn+1} | Run failure reason: {run.last_error}"
                        )
                    break;
            
            #self.set_rgb(command=MAGENTA+ALTERNATE_QUARTERS)
            #self.set_speech(filename="generic/waiting_door_open", wait_for_end_of=True)
            #self.set_face("charmie_face")
"""