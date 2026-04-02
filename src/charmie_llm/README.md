# LLM Package

This repository contains the source code, documentation, and testing suite for the **LLM** (Large Language Model) package. 

---

## Table of Contents

- [Project Overview](#project-overview)
- [Modes](#modes)
- [Functions](#functions)

---

## Project Overview

The **LLM Package** serves as a comprehensive toolkit to accelerate development with large-scale transformer-based models. The **LLM** package is designed to provide an interface for integrating with OpenAI's API for large language models (LLMs). Initially, the package interacts directly with OpenAI's API to generate responses from pre-trained models. However, our long-term goal is to deploy our own fine-tuned model on a cloud server to handle the same API requests. This transition allows us to have complete control over the model's performance, customization, test, and cost management. This repository contains the package implementation and serves as a foundation for both phases: 
- **Phase 1:** Integration with OpenAI API to Demo.
- **Phase 2:** Add Task Confirmation with OpenAI API.  
- **Phase 3:** Add GPSR with OpenAI API. 
- **Phase 4:** Add eGPSR with OpenAI API. 
- **Phase 5:** Deployment on our cloud infrastructure with a fine-tuned LLMs.

![LLM Overview](https://drive.google.com/file/d/1kAWLEP7YvKcFE-5glWn0-AN4rmHJq6y0/view?usp=drive_link)

---

## Modes

- **Demo**: Demo request to LLM.
- **Task Confirmation**: Confirm the human command to GPSR and EGPSR.
- **GPSR**: Also includes the functions used to GPSR and EGPSR.

---
## Functions
!!!Missing new functions!!!
 In the context of large language models (LLMs), tools (or functions) are extra abilities that the model can use to extend what it can do. Normally, an LLM processes text and generates responses based on patterns in language. However, tools allow the model to perform specific tasks beyond just generating text. In our case will change the robot behavior.
 
| Tool | Input | Output | Description | Tested|
|--|--|--|--|--|
| Talk | Speak:string | Val:bool | The function receives a string from LLM and put on peakers | ❌ |
| Go_To | Place:string | Val:bool | The function receives a string that contains the room or furniture the robot move to that place | ❌ |
| Search_Objects | Object:string | Coordinates:Point | The function receives a string that contains the object, the robot search for object and returns the coordinates | ❌ |
| Search_Person | Name:string | Coordinates:Point | The function receives a string that contains the person name, the robot search for object and returns the coordinates | ❌ |
| Pick_Objects | Object:string | Val:bool | The function receives a string that contains the object name, the robot pick that object | ❌ |
| Place_On | Furniture:string | Val:bool | The function receives a string that contains the furniture, the robot place the object | ❌ |
| Place_Next| Object:string | Val:bool | The function receives a string that contains the object, the robot place the pick object next to the other object | ❌ |
| Give_Object | void | Val:bool | The robot give the object to the person and ask to the person to pick the object | ❌ |
| Open_Door | Door:string | Val:bool | The function receives a string that contains the door name, the robot open that door | ❌ |
| Close_Door | Door:string | Val:bool | The function receives a string that contains the door name, the robot close that door | ❌ |
| Follow_Me | Place:string | Val:bool | The function receives a string that contains the room or furniture, the robot ask to the person to follow and the robot starts to moving to that place | ❌ |
| Follow_Person | void | Val:bool | With this function, the robot start to follow the person just in front | ❌ |
| Count_People | Room:string | Number:int, Val:bool | The function receives a string that contains the room, the robot goes to that room and start to count people. | ❌ |
| Count_Object | void | Number:int, Val:bool | The function receives a string that contains the room, the robot goes to that room and start to count people | ❌ |
| What_Time | void | Hour:int, Minutes:int, Val:bool | The function returns the current time | ❌ |
| What_Day | void | Day:int, Month:int, Year:int, Val:bool | The function returns the current day | ❌ |

