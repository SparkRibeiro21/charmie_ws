import json
from openai import OpenAI
import sys
from pathlib import Path

class LLM_confirmation_description:
    def __init__(self):
        # Define the instructions text

        # by using self.home it automatically adjusts to all computers home file, which may differ since it depends on the username on the PC
        self.home = str(Path.home())
        api_key_path = self.home+'/'+"charmie_ws/src/charmie_llm/charmie_llm/api_key/api_key.txt"
        self.confirm=False
        try:
            with open(api_key_path, "r") as file:
                self.api_key = file.read().strip()
            print(self.api_key)
            print(type(self.api_key))
        except FileNotFoundError:
            print(f"The file api_key.txt does not exist.")
            sys.exit()  # Ends the program if the file is not found

        instructions_text = """
        Your name is Charmie for the user, you are a robotic assistant.
        You are a speech-to-text debugger. 
        The messages you receive from the user will have to be simplified by you in the input (string) of the Task function. 
        After using the function, which is always mandatory, you must ask the user if is the task that you understand, if the user confirm, call the Confirm function.
        If the user deny, the user will repeat the sentence in the same message and call the Task function again with the task already corrected and ask again for confirmation.
        """
        #TODO add list of objects, furniture, rooms, names, etc...


        tools_description = [
        {
            "function": {
                "name": "Task",
                "description": "Called when the chat receive ",
                "parameters": {
                    "type": "object",
                    "properties": {"simplified": {"type": "string"}},
                    "required": ["simplified"],
                },
            },
            "type": "function",
        },
        {
            "function": {
                "name": "Confirm",
                "description": "Called when the user confirm the last Task submited",
            },
            "type": "function",
        }
        ]
        self.client = OpenAI(api_key=self.api_key) #TODO: CHANGE TO SECRETS
        
        vector_store = self.client.beta.vector_stores.create(name="Documentation")
        self.thread = self.client.beta.threads.create()
        # Ready the files for upload to OpenAI
        folder_path = self.home+"/charmie_ws/src/charmie_llm/charmie_llm/Files/"
        file_paths = ["tdp2023.pdf", "tdp2024.pdf","Actual Config.txt","Actual Team Members.txt", "Assistant Tasks.txt","Hardware Overview.txt"]  # List of your PDF files
        file_streams = [open(folder_path+path, "rb") for path in file_paths]

        # Use the upload and poll SDK helper to upload the files, add them to the vector store,
        # and poll the status of the file batch for completion.
        self.client.beta.vector_stores.file_batches.upload_and_poll(
        vector_store_id=vector_store.id, files=file_streams
        )
        self.assistant = self.client.beta.assistants.create(
            name="Charmie",
            instructions=instructions_text,
            model="gpt-3.5-turbo-0125",
            tools=tools_description,
            tool_resources={"file_search": {"vector_store_ids": [vector_store.id]}}, #TODO: check files.
        )
        
        self.tools = {
        "Task": self.Task,
        "Confirm": self.Confirm,
        }
    
    def run(self,command="What is the first capital of Portugal?"):
        max_turns=2
        for turn in range(max_turns):
            # Send user input as a message in the thread
            # Fetch the last message from the thread
            messages = self.client.beta.threads.messages.list(
                thread_id=self.thread.id,
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
                print(assistant_res)
                return assistant_res

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
                    print(args)
                    func_output = self.tools[tool.function.name](**args)

                    # OpenAI needs the output of the function call against the tool_call_id
                    func_tool_outputs.append(
                        {"tool_call_id": tool.id, "output": str(func_output)}
                    )

                # Submit the function call outputs back to OpenAI
                run = self.client.beta.threads.runs.submit_tool_outputs_and_poll(
                    thread_id=self.thread.id, run_id=run.id, tool_outputs=func_tool_outputs
                )

                # Continue the agent loop.
                # Agent will check the output of the function output submission as part of next iteration.
                continue

            # Handle errors if terminal state is "failed"
            else:
                if run.status == "failed":
                    print("ERRO LLM "+run.status) #TODO: add Log Node
                return "Info to Programmer: LLM Confirmation not working "+run.status
    
    def Task(self,simplified: str) -> bool:
        # do something
        self.confirm=False
        print("TASK: "+simplified+ " "+self.confirm)
        return True

    def Confirm(self) -> bool:
        
        self.confirm=True
        print("CONFIRM! "+self.confirm)
        # do something
        return True


