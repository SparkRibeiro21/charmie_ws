# TODO adicionar as outras std_functions

std_functions = [

# # set_speech(self, filename="", command="", quick_voice=False, show_in_face=False, long_pause_show_in_face=False, breakable_play=False, break_play=False, wait_for_end_of=True)
#     {
#         "name": "set_speech",
#         "type": "function",
#         "description": "Sets speech to be spoken by the robot",
#         "parameters": {
#             "type": "object",
#             "properties": {
#                 "filename": {
#                     "type": "string",
#                     "description": "The name of the file cointaining the speech to be played. Must be an empty string"
#                 },
#                 "command": {
#                     "type": "string",
#                     "description": "The message to be said by the robot"
#                 },
#                 "quick_voice": {
#                     "type": "boolean",
#                     "enum": [True], # forces the mdel to always choose True
#                     "description": "Whether to use quick voice mode. Must be True."
#                 },
#                 "show_in_face": {
#                     "type": "boolean",
#                     "enum": [False],
#                     "description": "Whether to show the speech in the robot's face. Must be False."
#                 },
#                 "long_pause_show_in_face": {
#                     "type": "boolean",
#                     "enum": [False],
#                     "description": "Whether to make long pauses before showing in the robot's face. Must be False."
#                 },
#                 "breakable_play": {
#                     "type": "boolean",
#                     "enum": [False],
#                     "description": "Whether the speech can be interrupted. Must be False."
#                 },
#                 "break_play": {
#                     "type": "boolean",
#                     "enum": [False],
#                     "description": "Whether to break the currentspeech play. Must be False."
#                 },
#                 "wait_for_end_of": {
#                     "type": "boolean",
#                     "enum": [True],
#                     "description": "Whether to wait for the end of the speech before proceeding. Must be True."
#                 }
#             }
#         }
#     },
# # 

# move_to_position(self, move_coords, print_feedback=True, feedback_freq=1.0, wait_for_end_of=True)
# TODO vai ser preciso um passo intermédio antes de chamar a std_function para converter o move_coords
    {
        "name": "move_to_room",
        "type": "function",
        "description": "Moves to a specified room in the house",
        "parameters": {
            "type": "object",
            "properties": {
                "room": {
                    "type": "string",
                    "enum": ["living room", "kitchen", "bedroom", "office", "dining room", "laundry room", "foyer", "bathroom"],
                    "description": "The room to move to"
                }
            }
        }
    },

    {
        "name": "move_to_furniture",
        "type": "function",
        "description": "Moves to a specified piece of furniture",
        "parameters": {
            "type": "object",
            "properties": {
                "furniture": {
                    "type": "string",
                    "enum": ["couch", "cabinet", "coffee table", "side table", "dining table", "trash bin", "couch", "bookcase", "shelf", "washing machine", "dishwasher", "pantry", "bed", "nightstand"],
                    "description": "The piece of furniture to move to"
                }
            }
        }
    },
    
    {
        "name": "move_to_person_through_name",
        "type": "function",
        "description": "Searches for a person in the surrounding environment with a specified name and moves to them",
        "parameters": {
            "type": "object",
            "properties": {
                "person_name": {
                    "type": "string",
                    "description": "The name of the person to find and move to"
                }
            }
        }

    },

    {
        "name": "move_to_person_with_pose",
        "type": "function",
        "description": "Searches for a person in the surrounding environment with a specified pose and moves to them",
        "parameters": {
            "type": "object",
            "properties": {
                "person_pose": {
                    "type": "string",
                    "description": "The pose of the person to find and move to"
                }
            }
        }
    },

    {
        "name": "move_to_person_with_clothing",
        "type": "function",
        "description": "Searches for a person in the surrounding environment with a specified clothing and moves to them",
        "parameters": {
            "type": "object",
            "properties": {
                "person_clothing": {
                    "type": "string",   
                    "description": "The characteristics of theclothing of the person to find and move to"
                }
            }
        }
    },

    {
        "name": "move_to_initial_person_position",
        "type": "function",
        "description": "Moves to the the person who made the request"
    },

# DEBUG ONLY (NOT YET THE CORRECT STANDARD FUNCTIONS AND PARAMETERS)
    {
        "name": "pick_object",
        "type": "function",
        "description": "Searches for an object in the surrounding environment and picks it up",
        "parameters": {
            "type": "object",
            "properties": {
                "name_of_object": {
                    "type": "string",
                    "description": "The name of the object or item to be picked up"
                }
            }
        }
    },
    
    {
        "name": "place_object_on_furniture",
        "type": "function",
        "description": "Moves to the room with a specified piece of furniture. Then moves to that furniture and places an object on it",
        "parameters": {
            "type": "object",
            "properties": {
                "name_of_object": {
                    "type": "string",
                    "description": "The name of the object to be placed"
                },
                "furniture": {
                    "type": "string",
                    "description": "The name of the furniture where the object should be placed"
                }
            }
        }
    },

    {
        "name": "hand_object_to_person",
        "type": "function",
        "description": "Hands an object to a person"
    }
]