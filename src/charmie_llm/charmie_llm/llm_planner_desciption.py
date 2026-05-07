import ollama
#from openai import OpenAI

import sys
from pathlib import Path

from charmie_llm.functions_list import std_functions

# TODO gerar ficheiros com exemplos (ver se o modelo responde melhor a exemplos in-prompt ou com file search)

# class LLM_planner_description:

#     # Initializinng the LLM (loads API key, creates client)
#     def __init__(self):

#         self.openai_model= "gpt-4o-mini-2024-07-18"

#         # # preparing the api key
#         # self.home = str(Path.home())
#         # api_key_path = self.home+'/'+"charmie_ws/src/charmie_llm/charmie_llm/api_key/api_key.txt"
#         # try:
#         #     with open(api_key_path, "r") as file:
#         #         self.api_key = file.read().strip()
#         #     #print(self.api_key)
#         #     #print(type(self.api_key))
#         # except FileNotFoundError:
#         #     print(f"The file api_key.txt does not exist.")
#         #     sys.exit()  # Ends the program if the file is not found

#         # # fetching the client with the api key
#         # self.client = OpenAI(api_key = self.api_key)

#     # sends a command to the LLM and gets the response (the generated plan)
#     def low_level_planner(self, request: str):
#         response = ""
#         # response = self.client.responses.create(
#         #     model=self.openai_model,
#         #     input=[
#         #         {
#         #             "role": "system",
#         #             "content": (
#         #                 "You are a low-level robot planner."
#         #                 "You will be given a high-level plan as a paragraph where each sentence is one step. "
#         #                 "Your job is to translate every step into one or more sequential function calls with the correct parameters. "
#         #                 "To talk to a person you need to move to their location first."

#         #                 "Follow these rules strictly: "
#         #                 "1. Process steps in order — do not skip or reorder them. "
#         #                 "2. The pick_object function already handles searching for the object, so if both appear, only call pick_object once. "
#         #                 # The move_to already handles figuring out the location of the target (person, furniture, object or room), so no need to specify that in the plan.
#         #                 "3. To interact with or hand something to a person, you must first call the appropriate move_to_person function. "
#         #                 "4. Use move_to_person_through_name when the person is identified by name. "
#         #                 "5. Use move_to_person_with_pose or move_to_person_with_clothing when the person is identified by appearance. "
#         #                 "6. Use move_to_initial_person_position to return to the person who made the original request. "
#         #                 "7. Use move_to_furniture when moving to a piece of furniture (e.g. cabinet, side table). "
#         #                 "8. Use move_to_room only when moving to a room without a specific furniture target. "
#         #                 # USE PICK_OBJECT TO SEARCH AND PICK
#         #                 "9. hand_object_to_person has no parameters — only call it after already being next to the person. "
#         #                 "10. To talk to a person you need to move to their location first."
#         #                 "11. Always complete ALL steps in the plan. Do not stop until every sentence has been translated into function calls."
#         #                 )
#         #         },
#         #         {
#         #             "role": "user",
#         #             "content": (
#         #                 "The high-level plan is: " + request
#         #             )
#         #         }
#         #     ],
#         #     # TODO: juntar histórico de conversa (previous_response_id)
#         #     tools=std_functions,
#         #     parallel_tool_calls= True,
#         #     tool_choice="auto"
#         #     # tool_choice= "required"  # the model is required to use the function call in the response
#         # )

#         return response
    
#     def high_level_planner(self, request: str):

#         response = self.client.responses.create(
#             model=self.openai_model,
#             input=[
#                 {
#                     "role": "system",
#                     "content": (
#                         "You are a high-level robot planner."
#                         "You will be given a task and must break it down into a clear, sequential high-level plan. "
#                         "Your output must be a high-level plan in the form of a paragraph describing the plan. Each step should be a sentence. "
#                         "Write in first person as if you are the robot executing the plan (e.g., 'I will move to the kitchen'). "
#                         "Keep each step simple and include only necessary information. "

#                         "Follow these rules strictly: "
#                         "1. People do not move unless explicitly instructed. "
#                         "2. Do not assume any furniture's location. "
#                         "3. You must be at the same location as a person to talk to or hand something to them. "
#                         "4. You must physically move to a location before interacting with anything or anyone there. "
#                         "5. When delivering to a named person, include a step to move to that person by name, then a step to hand the object. "
#                         "6. The environment will be prepared so that all necessary objects are easily visible and accessible (nno need to search for objects or open doors/drawers). "     
#                         # "1. Your starting position is always the living room. "
#                         # "2. Never write a separate 'search for' or 'find' step for objects — picking up an object already includes finding it. "
#                         # "6. When delivering to a named person, include a step to move to that person by name, then a step to hand the object. "           
#                     )
#                 },
#                 {
#                     "role": "user",
#                     "content": (
#                         "The request is: " + request
#                     )
#                 }
#             ],
#         )

#         return response.output_text
    
#     # def handle_request(self, request: str):

#     #     high_level_plan = self.high_level_planner(request)
#     #     print("High-level plan: ", high_level_plan.output_text)

#     #     low_level_plan = self.low_level_planner(high_level_plan.output_text)
#     #     print("Low-level plan: ", low_level_plan)

#     #     return low_level_plan


class Ollama_planner_description:

    # Initializinng the LLM (loads API key, creates client)
    def __init__(self):

        self.hlp_creative_model= "llama3.2:3b"

        print("HLP model initialized")

    def high_level_planner(self, request: str):

        response = ollama.chat(
            model= self.hlp_creative_model,
            messages= [{"role": "system",
                        "content": (
                            "You are a robot task planner. Convert a command into atomic steps separated by semicolons (;). "
                            "Write in first person. One paragraph only. No explanations or reasoning.\n\n"

                            "Follow these rules strictly:\n"
                            "1. Never invent objects, people, or places not in the command.\n"
                            "2. Always move before interacting with anything.\n"
                            "3. After moving to a location, always look for the object or person before touching or addressing them.\n"
                            "4. The locutor is in the instruction point If you need to talk to them or hand them an object, move to them first.\n"
                            "5. The verbs guide, escort, take mean the same. \n"
                            "6. The verbs follow and stick with mean the same. \n"
                            "7. Use only these verbs: move to, look for, pick, place, hand, follow, guide, greet, count, speak, tell.\n\n"

                            "EXAMPLES:\n"
                            "Command: Retrieve a dish from the refrigerator and subsequently position it on the storage rack.\n"
                            "Answer: I will move to the refrigerator; Then, I will look for the dish; I will pick up the dish; I will move to the storage rack; I will place the dish on the storage rack.\n\n"

                            "Command: Could you take a fruit from the armchair and bring it to Robin in the bathroom\n"
                            "Answer: First, I will move to the armchair; I will look for the fruit; I will pick up the fruit; I will move to the bathroom; I will look for Robin; I will move towards Robin; I will hand the fruit to Robin.\n\n"

                            "Command: go meet Wyatt in the living room and take them to the bedroom\n"
                            "Answer: I will move to the living room; I will look for Wyatt; If found, I will move towards Wyatt; I will greet Wyatt; I will guide Wyatt to the bedroom.\n\n"

                            "Command: please proceed to the living room, introduce yourself to the person wearing a black jacket, and stick with them\n"
                            "Answer: I will move to the living room; I will look for the person wearing a black jacket; If found, I will move towards them; I will greet the person wearing a black jacket; I will follow the person wearing a black jacket.\n\n"

                            "Command: kindly ascertain the number of snacks on the cabinet and inform me\n"
                            "Answer: I will move to the cabinet; I will count the snacks on the cabinet; I will move towards you; Lastly, I will tell you the result.\n\n"

                            "Command: Could you tell me what is the heaviest object on the shelf\n"
                            "Answer: I will move to the shelf; I will look for the heaviest object; Then, I will move towards you; I will tell you the result.\n\n"

                            "Command: Tell your team's name to the person pointing left in the kitchen\n"
                            "Answer: I will move to the kitchen;I will look for the person pointing left; After that, I will move towards that person; I will tell the person pointing to the left my team's name.\n\n"

                            "Command: locate the waving person in the hallway, tell them what day tomorrow is and then and take them from hallway to the sofa\n"
                            "Answer: I will move to the hallway; I will look for the waving person; I will move towards the waving person; Then, I will tell the waving person what day tomorrow is; And then  I will guide them from hallway to the sofa.\n"
                        )},
                      {"role":"user",
                       "content":f"""
                        Command: "{request}"

                        Answer:
                        """}]
        )

        print("1st Plan Generated:", response["message"]["content"])

        return response["message"]["content"]

    def low_level_planner(self, request: str):

        response = ollama.chat(
            model= self.hlp_creative_model,
            messages=[{"role": "system",
                        "content": (
                            "You are a robot low-level planner."
                            "Your job is to extract the relevant information from the command."
                            "You must identify the action and the parameter."
                            "Write the output in this format: 'action-parameter'"

                            "Follow these rules strictly:\n"
                            #TODO Ver se compensa ter os diferentes tipos de move aqui ou se é melhor fazer um prompt para ação geral e depois voltar a fazer para o tipo de ação+parametro
                            "1. The only available actions are: move_to_furniture, move_to_room, move_to_person, look_for_object, look_for_person_pose, pick_up_object, hand_object, place_object.\n"
                            "2. If the action is neither of the previous ones, return: 'ERROR'."

                            "EXAMPLES:\n"
                            "Command: I will move to the bed; \n"
                            "Answer: move_to_furniture-bed\n\n"

                            "Command: I will move to the kitchen; \n"
                            "Answer: move_to_room-kitchen\n\n"

                            "Command: I will look for the lemon; \n"
                            "Answer: look_for_object-lemon\n\n"

                            "Command: I will look for the standing person; \n"
                            "Answer: look_for_person_pose-standing\n\n"

                            "Command: I will move towards Robin; \n"
                            "Answer: move_to_person-Robin\n\n"

                            "Command: I will hand the tropical juice to Gary. \n"
                            "Answer: hand_object-Gary\n\n"

                            "Command: I will place the biscuits in the shelf. \n"
                            "Answer: place_object-shelf\n\n"


                        )},
                      {"role":"user",
                       "content":f"""
                        Command: "{request}"

                        Answer:
                        """}]
        )

        print("Action Generated:", response["message"]["content"])

        return response["message"]["content"]
    