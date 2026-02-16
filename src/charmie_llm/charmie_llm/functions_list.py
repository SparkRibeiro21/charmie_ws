# TODO adicionar as outras std_functions

std_functions = [

# set_speech(self, filename="", command="", quick_voice=False, show_in_face=False, long_pause_show_in_face=False, breakable_play=False, break_play=False, wait_for_end_of=True)
    {
        "name": "set_speech",
        "type": "function",
        "description": "Sets speech to be spoken by the robot",
        "parameters": {
            "type": "object",
            "properties": {
                "filename": {
                    "type": "string",
                    "description": "The name of the file cointaining the speech to be played. Must be an empty string"
                },
                "command": {
                    "type": "string",
                    "description": "The message to be said by the robot"
                },
                "quick_voice": {
                    "type": "boolean",
                    "enum": [True], # forces the mdel to always choose True
                    "description": "Whether to use quick voice mode. Must be True."
                },
                "show_in_face": {
                    "type": "boolean",
                    "enum": [False],
                    "description": "Whether to show the speech in the robot's face. Must be False."
                },
                "long_pause_show_in_face": {
                    "type": "boolean",
                    "enum": [False],
                    "description": "Whether to make long pauses before showing in the robot's face. Must be False."
                },
                "breakable_play": {
                    "type": "boolean",
                    "enum": [False],
                    "description": "Whether the speech can be interrupted. Must be False."
                },
                "break_play": {
                    "type": "boolean",
                    "enum": [False],
                    "description": "Whether to break the currentspeech play. Must be False."
                },
                "wait_for_end_of": {
                    "type": "boolean",
                    "enum": [True],
                    "description": "Whether to wait for the end of the speech before proceeding. Must be True."
                }
            }
        }
    },
# 

# move_to_position(self, move_coords, print_feedback=True, feedback_freq=1.0, wait_for_end_of=True)
# TODO vai ser preciso um passo intermédio antes de chamar a std_function para converter o move_coords
    {
        "name": "move_to_position",
        "type": "function",
        "description": "Moves to a specified location",
        "parameters": {
            "type": "object",
            "properties": {
                "move_coords": {
                    "type": "string",
                    "description": "The location or object to move to"
                },
                "print_feedback": {
                    "type": "boolean",
                    "enum": [True, False],
                    "description": "Whether to print feedback of position during movement"
                },
                "feedback_freq": {
                    "type": "number",
                    "enum": [1.0],
                    "description": "Frequency (in seconds) of feedback printing. Must be 1.0."
                },
                "wait_for_end_of": {
                    "type": "boolean",
                    "enum": [True],
                    "description": "Whether to wait for the end of the movement before proceeding. Must be True."
                }
            }
        }
    }
]