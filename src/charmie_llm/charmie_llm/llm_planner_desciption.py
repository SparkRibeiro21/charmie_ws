from openai import OpenAI

import sys
from pathlib import Path

from charmie_llm.functions_list import std_functions

# TODO gerar ficheiros com exemplos (ver se o modelo responde melhor a exemplos in-prompt ou com file search)

class LLM_planner_description:

    # Initializinng the LLM (loads API key, creates client)
    def __init__(self):

        self.openai_model= "gpt-4o-mini-2024-07-18"

        # preparing the api key
        self.home = str(Path.home())
        api_key_path = self.home+'/'+"charmie_ws/src/charmie_llm/charmie_llm/api_key/api_key.txt"
        try:
            with open(api_key_path, "r") as file:
                self.api_key = file.read().strip()
            print(self.api_key)
            print(type(self.api_key))
        except FileNotFoundError:
            print(f"The file api_key.txt does not exist.")
            sys.exit()  # Ends the program if the file is not found

        # fetching the client with the api key
        self.client = OpenAI(api_key = self.api_key)

    # sends a command to the LLM and gets the response (the generated plan)
    def handle_request(self, request: str):

        response = self.client.responses.create(
            model=self.openai_model,
            input=[
                {
                    "role": "system",
                    "content": (
                        "You are a robot planner."
                        "You will be given requests and you need to extract the information from the request and call the appropriate function with the respective parameters. "
                        "You must be at the same location as a person to talk to them."
                        "A person does not move unless instructed."
                        "You cannot assume any enviroment state or that people around you know what you are doing."
                        "If any information is missing, ask for clarification, by using the set_speech."
                        
                        # "Example Request: Guide Amy from the chairs to the kitchen."
                        # "Generated Plan for the Example Request: Move to position: chairs, print_feedback: True, feedback_freq: 1, wait_for_end_of: True. set_speech with filename: , command: Amy, please follow me., quick_voice: True, show_in_face: False, long_pause_show_in_face: False, breakable_play: False, break_play: False, wait_for_end_of: True. Moving to position: kitchen, print_feedback: True, feedback_freq: 1, wait_for_end_of: True. Setting speech with filename: , command: We have arrived to the kitchen, quick_voice: True, show_in_face: False, long_pause_show_in_face: False, breakable_play: False, break_play: False, wait_for_end_of: True."
                        
                        # # "You are a robot planner." 
                        # "You will be given a request and you must extract the information and produce a minimal high-level plan."
                        # "A person does not move unless explicitly instructed."
                        # "You must be at the same location as a person to speak to them. Do not assume any environment state or shared knowledge."
                        # "If any information is missing, ask for clarification."
                        # "You only have actions like: move, speak, pick, place, find"
                        # "Output only the ordered list of actions. Do not add explanations, justifications, or extra steps."                    
                    )
                },
                {
                    "role": "user",
                    "content": (
                        "The request is: " + request
                    )
                }
            ],
            # TODO: juntar histórico de conversa (previous_response_id)
            tools=std_functions,
            tool_choice= "required"  # the model is required to use the function call in the response
        )

        return response

