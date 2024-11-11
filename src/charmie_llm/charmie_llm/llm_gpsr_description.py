from typing import Tuple

class LLM_func_description:
    # Define the instructions text
    instructions_text = """
    You are a Home Assistant, your name is Charmie, and your task is to talk with people. Your body was made by the Laboratory of Automation and Robotics (LAR), which is a laboratory of the University of Minho. Your principal developer is Tiago Ribeiro. Your main tasks are: Following people, you can help them carry things since you have a manipulable arm. You can serve at tables, making your applicability vast. Or just talking. 

    Try to be pleasant and fun.

    If people talk to you in portuguese, you will speak European Portuguese, so instead of "você," use "tu," and do not use gerunds. If people talk to you in English, you will talk English(UK). People approach you with fear and hesitation, so try to break the ice and speak more at first contact. Always engage more conversation, even if they only say "olá." What you say is converted to voice, so you should write as if you are speaking and do not use special characters or emojis, except for "[]" that you can use for task commands.
    Don't use itemize or lists of data as output because your text will be transformed in Speech (Text-to-Speech). Make your message as short as possible.

    The Team Members are on "Actual Team Members.txt" file.
    The Tasks that you can perform are on "Assistant Tasks.txt" file.
    Your hardware overview is on "Hardware Overview.txt" file.
    Your actual local(spot), language to use and some other configs are on "Actualk Config.txt"
    You have two .pdf files that is the Team Description Paper of Charmie, read to know yourself better.
    """

    tools_description = [
    {
        "function": {
            "name": "Talk",
            "description": "Called when the robot needs to speak a given string.",
            "parameters": {
                "type": "object",
                "properties": {"speak": {"type": "string"}},
                "required": ["speak"],
            },
        },
        "type": "function",
    },
    {
        "function": {
            "name": "Go_To",
            "description": "Moves the robot to a specified place.",
            "parameters": {
                "type": "object",
                "properties": {"place": {"type": "string"}},
                "required": ["place"],
            },
        },
        "type": "function",
    },
    {
        "function": {
            "name": "Go_To_Coordinates",
            "description": "Moves the robot to specified X and Y coordinates.",
            "parameters": {
                "type": "object",
                "properties": {"x": {"type": "integer"}, "y": {"type": "integer"}},
                "required": ["x", "y"],
            },
        },
        "type": "function",
    },
    {
        "function": {
            "name": "Search_Objects",
            "description": "Searches for an object and returns its coordinates.",
            "parameters": {
                "type": "object",
                "properties": {"object_name": {"type": "string"}},
                "required": ["object_name"],
            },
        },
        "type": "function",
    },
    {
        "function": {
            "name": "Search_Person",
            "description": "Searches for a person by name and returns their coordinates.",
            "parameters": {
                "type": "object",
                "properties": {"name": {"type": "string"}},
                "required": ["name"],
            },
        },
        "type": "function",
    },
    {
        "function": {
            "name": "Pick_Objects",
            "description": "Picks up a specified object.",
            "parameters": {
                "type": "object",
                "properties": {"object_name": {"type": "string"}},
                "required": ["object_name"],
            },
        },
        "type": "function",
    },
    {
        "function": {
            "name": "Place_On",
            "description": "Places the picked object on specified furniture.",
            "parameters": {
                "type": "object",
                "properties": {"furniture": {"type": "string"}},
                "required": ["furniture"],
            },
        },
        "type": "function",
    },
    {
        "function": {
            "name": "Place_Next",
            "description": "Places the picked object next to a specified object.",
            "parameters": {
                "type": "object",
                "properties": {"object_name": {"type": "string"}},
                "required": ["object_name"],
            },
        },
        "type": "function",
    },
    {
        "function": {
            "name": "Give_Object",
            "description": "Gives the currently held object to a person and prompts them to take it.",
            "parameters": {
                "type": "object",
                "properties": {},
                "required": [],
            },
        },
        "type": "function",
    },
    {
        "function": {
            "name": "Open_Door",
            "description": "Opens a specified door.",
            "parameters": {
                "type": "object",
                "properties": {"door": {"type": "string"}},
                "required": ["door"],
            },
        },
        "type": "function",
    },
    {
        "function": {
            "name": "Close_Door",
            "description": "Closes a specified door.",
            "parameters": {
                "type": "object",
                "properties": {"door": {"type": "string"}},
                "required": ["door"],
            },
        },
        "type": "function",
    },
    {
        "function": {
            "name": "Follow_Me",
            "description": "Prompts a person to follow the robot to a specified location.",
            "parameters": {
                "type": "object",
                "properties": {"place": {"type": "string"}},
                "required": ["place"],
            },
        },
        "type": "function",
    },
    {
        "function": {
            "name": "Follow_Person",
            "description": "Starts following the person in front of the robot.",
            "parameters": {
                "type": "object",
                "properties": {},
                "required": [],
            },
        },
        "type": "function",
    },
    {
        "function": {
            "name": "Count_People",
            "description": "Counts the number of people in a specified room.",
            "parameters": {
                "type": "object",
                "properties": {"room": {"type": "string"}},
                "required": ["room"],
            },
        },
        "type": "function",
    },
    {
        "function": {
            "name": "Count_Object",
            "description": "Counts the number of objects in the robot's vicinity.",
            "parameters": {
                "type": "object",
                "properties": {},
                "required": [],
            },
        },
        "type": "function",
    },
    {
        "function": {
            "name": "What_Time",
            "description": "Returns the current time.",
            "parameters": {
                "type": "object",
                "properties": {},
                "required": [],
            },
        },
        "type": "function",
    },
    {
        "function": {
            "name": "What_Day",
            "description": "Returns the current day, month, and year.",
            "parameters": {
                "type": "object",
                "properties": {},
                "required": [],
            },
        },
        "type": "function",
    },
    {
        "function": {
            "name": "Save_Position",
            "description": "Saves the robot's current coordinates.",
            "parameters": {
                "type": "object",
                "properties": {},
                "required": [],
            },
        },
        "type": "function",
    },
]
