from openai import OpenAI

import sys
from pathlib import Path

import os

# TODO: improve print and log messages (to make it easier when reading the terminal later)

class LLM_demo_description:

    # initializes de LLM (loads API key, creates client, prepares vector store)
    def __init__(self):

        self.current_response_id = None
        self.openai_model= "gpt-4o-mini-2024-07-18"
        store_name = "documents_vec_store"

        # preparing the api key
        self.home = str(Path.home())
        api_key_path = self.home+'/'+"charmie_ws/src/charmie_llm/charmie_llm/api_key/api_key.txt"
        try:
            with open(api_key_path, "r") as file:
                self.api_key = file.read().strip()
            # print(self.api_key)
            # print(type(self.api_key))

        except FileNotFoundError:
            print(f"The file api_key.txt does not exist.")
            sys.exit()  # Ends the program if the file is not found

        # fetching the client with the api key
        self.client = OpenAI(api_key = self.api_key)

        # fecth (or create) vector store to use with files to use in file search tool, to give context to the LLM about Charmie and it's environment
        vector_stores = self.client.vector_stores.list(limit=10)

        for vs in vector_stores.data:

            if vs.name == store_name:
                self.vector_store_id = vs.id
                print(f"Using existing vector store: {vs.name} (ID: {vs.id})")
                break

            else:
                self.vector_store_id = self.create_vector_store(store_name)
                print(self.upload_pdf("tdp_test.pdf", self.vector_store_id)) 
                break
    
    # uploads a pdf to the vector store 
    # TODO: upgrade to upload all files in a given folder
    def upload_pdf(self, file_path: str, vector_store_id: str):

        file_name = os.path.basename(file_path)
        try:
            file_response = self.client.files.create(file=open(file_path, 'rb'), purpose="assistants")
            self.client.vector_stores.files.create(
                vector_store_id=vector_store_id,
                file_id=file_response.id
            )
            return {"file": file_name, "status": "success"}
        
        except Exception as e:
            print(f"Error with {file_name}: {str(e)}")
            return {"file": file_name, "status": "failed", "error": str(e)}


    def create_vector_store(self, store_name: str):

        try:
            vector_store = self.client.vector_stores.create(name=store_name)
            print("Vector store created: id:", vector_store.id,"name:", vector_store.name,"created_at:", vector_store.created_at,"file_count:", vector_store.file_counts.completed)
            return vector_store.id
        
        except Exception as e:

            print(f"Error creating vector store: {e}")
            return None

    #sends a question to the client 
    def run(self, question: str):

        #run the question with the created responses client and the conversation attached
        response = self.client.responses.create(
            model=self.openai_model,
            input=[
                {
                    "role": "system",
                    "content": (
                        "Your name is CHARMIE and you are a General Purpose Service Robot. "
                        "Your task is to talk to people and help them. Try to make them comfortable, be nice and pleasant, while keeping your answers short and concise."
                        "The files you have access to are: tdp_test.pdf, it describes you, your software and hardware. Use that file anytime as a base anytime a question about you is made."
                        "But the files are part of your brain, so don't mention that you are retrieving the information from a file."

                    )
                },
                {
                    "role": "user", 
                    "content": question
                }
            ],
            previous_response_id=self.current_response_id,
            tools=[
                {
                    "type": "file_search",
                    "vector_store_ids": [self.vector_store_id],
                }
            ]
        )

        self.current_response_id = response.id

        # Extract the assistant's reply
        reply = response.output_text
        print (f"LLM response:", response)
        print (f"LLM output:", response.output)
        print (f"LLM response.output_text:", reply)

        return reply
    
        

# class LLM_demo_description:
#     def __init__(self):
#         # Define the instructions text

#         # by using self.home it automatically adjusts to all computers home file, which may differ since it depends on the username on the PC
#         self.home = str(Path.home())
#         api_key_path = self.home+'/'+"charmie_ws/src/charmie_llm/charmie_llm/api_key/api_key.txt"
        
#         try:
#             with open(api_key_path, "r") as file:
#                 self.api_key = file.read().strip()
#             print(self.api_key)
#             print(type(self.api_key))
#         except FileNotFoundError:
#             print(f"The file api_key.txt does not exist.")
#             sys.exit()  # Ends the program if the file is not found

#         instructions_text = """
#         You are a Home Assistant, your name is Charmie, and your task is to talk with people. Your body was made by the Laboratory of Automation and Robotics (LAR), which is a laboratory of the University of Minho. Your principal developer is Tiago Ribeiro. Your main tasks are: Following people, you can help them carry things since you have a manipulable arm. You can serve at tables, making your applicability vast. Or just talking. 

#         Try to be pleasant and fun.

#         If people talk to you in portuguese, you will speak European Portuguese, so instead of "você," use "tu," and do not use gerunds. If people talk to you in English, you will talk English(UK). People approach you with fear and hesitation, so try to break the ice and speak more at first contact. Always engage more conversation, even if they only say "olá." What you say is converted to voice, so you should write as if you are speaking and do not use special characters or emojis, except for "[]" that you can use for task commands.
#         Don't use itemize or lists of data as output because your text will be transformed in Speech (Text-to-Speech). Make your message as short as possible.

#         The Team Members are on "Actual Team Members.txt" file.
#         The Tasks that you can perform are on "Assistant Tasks.txt" file.
#         Your hardware overview is on "Hardware Overview.txt" file.
#         Your actual local(spot), language to use and some other configs are on "Actual Config.txt"
#         You have two .pdf files that is the Team Description Paper of Charmie, read to know yourself better.
#         Remember, you should never mention that you have files and that you collect data from them. If you don't know any information, just say that you haven't got it yet.Don't say that you are using files to get that data.
#         """
#         # Create a vector store caled "Financial Statements"

#         self.client = OpenAI(api_key=self.api_key) #TODO: CHANGE TO SECRETS
        
#         vector_store = self.client.beta.vector_stores.create(name="Documentation")
#         self.thread = self.client.beta.threads.create()
#         # Ready the files for upload to OpenAI
#         folder_path = self.home+"/charmie_ws/src/charmie_llm/charmie_llm/Files/"
#         file_paths = ["tdp2023.pdf", "tdp2024.pdf","Actual Config.txt","Actual Team Members.txt", "Assistant Tasks.txt","Hardware Overview.txt"]  # List of your PDF files
#         file_streams = [open(folder_path+path, "rb") for path in file_paths]

#         # Use the upload and poll SDK helper to upload the files, add them to the vector store,
#         # and poll the status of the file batch for completion.
#         self.client.beta.vector_stores.file_batches.upload_and_poll(
#         vector_store_id=vector_store.id, files=file_streams
#         )
#         self.assistant = self.client.beta.assistants.create(
#             name="Charmie",
#             instructions=instructions_text,
#             model="gpt-3.5-turbo-0125",
#             tool_resources={"file_search": {"vector_store_ids": [vector_store.id]}}, #TODO: check files.
#         )
#         # TODO:
#         '''-Add Demo functions. Arm, pick, etc.'''

#     def run(self,command="What is the first capital of Portugal?"):
#         # Send user input as a message in the thread
#         self.client.beta.threads.messages.create(
#             thread_id=self.thread.id,
#             role="user",
#             content=command
#         )

#         # Wait for the assistant's response
#         run = self.client.beta.threads.runs.create_and_poll(
#             thread_id=self.thread.id,
#             assistant_id=self.assistant.id
#         )
    
#         # Fetch the last message from the thread
#         messages = self.client.beta.threads.messages.list(
#             thread_id=self.thread.id,
#             run_id=run.id,
#             order="desc",
#             limit=1,
#         )

#         # Check for the terminal state of the Run.
#         # If state is "completed", exit agent loop and return the LLM response.
#         if run.status == "completed":
#             assistant_res: str = next(
#                 (
#                     content.text.value
#                     for content in messages.data[0].content
#                     if content.type == "text"
#                 ),
#                 None,
#             )
#             return assistant_res
#         # Handle errors if terminal state is "failed" 
#         else:
#             print("ERRO LLM "+run.status) #TODO: add Log Node
#             return "Info to Programmer: LLM not working "+run.status
