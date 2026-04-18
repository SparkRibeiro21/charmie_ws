#from openai import OpenAI

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
        gpsr_files_folder_path = self.home+'/'+"charmie_ws/src/configuration_files/gpsr_files/"
        environment_files_folder_path = self.home+'/'+"charmie_ws/src/configuration_files/"
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
                # print(f"Using existing vector store: {vs.name} (ID: {vs.id})")
                break

            else:
                self.vector_store_id = self.create_vector_store(store_name)
                # print(self.upload_all_files(gpsr_files_folder_path, self.vector_store_id)) 
                # print(self.upload_all_files(environment_files_folder_path, self.vector_store_id)) 
                break
    
    # # uploads a pdf to the vector store 
    # # TODO: upgrade to upload all files in a given folder
    # def upload_all_files(self, folder_path):
        
    #     for filename in os.listdir(folder_path):
    #         file_path = os.path.join(folder_path, filename)
    #         if os.path.isfile(file_path):
    #             result = self.upload_file(file_path)
    #             print(result)

    # def upload_file(self, file_path: str):

    #     file_name = os.path.basename(file_path)

    #     try:
    #         # For JSON/TXT files, read as text first
    #         ext = os.path.splitext(file_name)[1].lower()
    #         if ext in [".txt", ".json"]:
    #             with open(file_path, "r", encoding="utf-8") as f:
    #                 content = f.read()
    #             # Upload text content as a file
    #             temp_path = file_path + ".tmp.txt"
    #             with open(temp_path, "w", encoding="utf-8") as tmpf:
    #                 tmpf.write(content)
    #             file_response = self.client.files.create(file=open(temp_path, 'rb'), purpose="assistants")
    #             os.remove(temp_path)
    #         else:
    #             # Upload other files directly (PDF, etc.)
    #             file_response = self.client.files.create(file=open(file_path, 'rb'), purpose="assistants")

    #         self.client.vector_stores.files.create(
    #             vector_store_id=self.vector_store_id,
    #             file_id=file_response.id
    #         )

    #         return {"file": file_name, "status": "success"}

    #     except Exception as e:
    #         return {"file": file_name, "status": "failed", "error": str(e)}
        
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
                        "The files you have access to are: Identity.txt, it describes you and you role. Use that file anytime as a base anytime a question about you is made."
                        "The json files with the furniture, objects, rooms and names of the environment you are in, use them to get information about the environment and answer questions regarding it. The Hardware_Description file has information about your hardware capabilities, use it to answer questions about what you can do and how you can help. The Institutional_background_and_team.txt file has information about your institution and team, use it to answer questions regarding that."
                        "But the files are part of your brain, so don't mention that you are retrieving the information from a file."
                        "Keep your answers short. Answer only to what is asked, don't add extra information."
                        "Your output will be converted to speech, so write as if you are speaking, and do not use special characters or emojis or itemized lists. If you don't know the answer to a question, just say that you don't know, or that you haven't got that information yet."
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
        # print (f"LLM response:", response)
        # # print (f"LLM output:", response.output)
        # print (f"LLM response.output_text:", reply)

        return reply
