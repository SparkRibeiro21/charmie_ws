#from openai import OpenAI

import sys
from pathlib import Path

import os
import json
import numpy as np

import ollama
from torch import cosine_similarity

# TODO: improve print and log messages (to make it easier when reading the terminal later)

SCRIPT_DIR = Path(__file__).resolve().parent
from pathlib import Path

COMP_DOCUMENTATION_DIR = str(SCRIPT_DIR.parents[1] / "configuration_files" / "gpsr_files" / "Internal_knowledge" / "Competition")
DEMO_DOCUMENTATION_DIR = str(SCRIPT_DIR.parents[1] / "configuration_files" / "gpsr_files" / "Internal_knowledge" / "Demonstration")
CONFIG_FILES_DIR       = str(SCRIPT_DIR.parents[1] / "configuration_files")

COMP_EMBEDDINGS_FILE   = str(SCRIPT_DIR / "Embeddings_For_RAG" / "embeddings_comp.npz")
DEMO_EMBEDDINGS_FILE   = str(SCRIPT_DIR / "Embeddings_For_RAG" / "embeddings_demo.npz")

LLM_MODEL_NAME = "llama3.2:3b"
EMBEDDINGS_MODEL_NAME = "nomic-embed-text"

COMP_CHUNK_SIZE = 100
DEMO_CHUNK_SIZE = 250

COMP_TOP_K = 5
DEMO_TOP_K = 3




# class LLM_demo_description:

#     # initializes de LLM (loads API key, creates client, prepares vector store)
#     def __init__(self):

#         self.current_response_id = None
#         self.openai_model= "gpt-4o-mini-2024-07-18"
#         store_name = "documents_vec_store"

#         # preparing the api key
#         self.home = str(Path.home())
#         api_key_path = self.home+'/'+"charmie_ws/src/charmie_llm/charmie_llm/api_key/api_key.txt"
#         gpsr_files_folder_path = self.home+'/'+"charmie_ws/src/configuration_files/gpsr_files/"
#         environment_files_folder_path = self.home+'/'+"charmie_ws/src/configuration_files/"
#         try:
#             with open(api_key_path, "r") as file:
#                 self.api_key = file.read().strip()
#             # print(self.api_key)
#             # print(type(self.api_key))

#         except FileNotFoundError:
#             print(f"The file api_key.txt does not exist.")
#             sys.exit()  # Ends the program if the file is not found

#         # fetching the client with the api key
#         self.client = OpenAI(api_key = self.api_key)

#         # fecth (or create) vector store to use with files to use in file search tool, to give context to the LLM about Charmie and it's environment
#         vector_stores = self.client.vector_stores.list(limit=10)

#         for vs in vector_stores.data:

#             if vs.name == store_name:
#                 self.vector_store_id = vs.id
#                 # print(f"Using existing vector store: {vs.name} (ID: {vs.id})")
#                 break

#             else:
#                 self.vector_store_id = self.create_vector_store(store_name)
#                 # print(self.upload_all_files(gpsr_files_folder_path, self.vector_store_id)) 
#                 # print(self.upload_all_files(environment_files_folder_path, self.vector_store_id)) 
#                 break
    
#     # # uploads a pdf to the vector store 
#     # # TODO: upgrade to upload all files in a given folder
#     # def upload_all_files(self, folder_path):
        
#     #     for filename in os.listdir(folder_path):
#     #         file_path = os.path.join(folder_path, filename)
#     #         if os.path.isfile(file_path):
#     #             result = self.upload_file(file_path)
#     #             print(result)

#     # def upload_file(self, file_path: str):

#     #     file_name = os.path.basename(file_path)

#     #     try:
#     #         # For JSON/TXT files, read as text first
#     #         ext = os.path.splitext(file_name)[1].lower()
#     #         if ext in [".txt", ".json"]:
#     #             with open(file_path, "r", encoding="utf-8") as f:
#     #                 content = f.read()
#     #             # Upload text content as a file
#     #             temp_path = file_path + ".tmp.txt"
#     #             with open(temp_path, "w", encoding="utf-8") as tmpf:
#     #                 tmpf.write(content)
#     #             file_response = self.client.files.create(file=open(temp_path, 'rb'), purpose="assistants")
#     #             os.remove(temp_path)
#     #         else:
#     #             # Upload other files directly (PDF, etc.)
#     #             file_response = self.client.files.create(file=open(file_path, 'rb'), purpose="assistants")

#     #         self.client.vector_stores.files.create(
#     #             vector_store_id=self.vector_store_id,
#     #             file_id=file_response.id
#     #         )

#     #         return {"file": file_name, "status": "success"}

#     #     except Exception as e:
#     #         return {"file": file_name, "status": "failed", "error": str(e)}
        
#     def create_vector_store(self, store_name: str):

#         try:
#             vector_store = self.client.vector_stores.create(name=store_name)
#             print("Vector store created: id:", vector_store.id,"name:", vector_store.name,"created_at:", vector_store.created_at,"file_count:", vector_store.file_counts.completed)
#             return vector_store.id
        
#         except Exception as e:

#             print(f"Error creating vector store: {e}")
#             return None

#     #sends a question to the client 
#     def run(self, question: str):

#         #run the question with the created responses client and the conversation attached
#         response = self.client.responses.create(
#             model=self.openai_model,
#             input=[
#                 {
#                     "role": "system",
#                     "content": (
#                         "Your name is CHARMIE and you are a General Purpose Service Robot. "
#                         "Your task is to talk to people and help them. Try to make them comfortable, be nice and pleasant, while keeping your answers short and concise."
#                         "The files you have access to are: Identity.txt, it describes you and you role. Use that file anytime as a base anytime a question about you is made."
#                         "The json files with the furniture, objects, rooms and names of the environment you are in, use them to get information about the environment and answer questions regarding it. The Hardware_Description file has information about your hardware capabilities, use it to answer questions about what you can do and how you can help. The Institutional_background_and_team.txt file has information about your institution and team, use it to answer questions regarding that."
#                         "But the files are part of your brain, so don't mention that you are retrieving the information from a file."
#                         "Keep your answers short. Answer only to what is asked, don't add extra information."
#                         "Your output will be converted to speech, so write as if you are speaking, and do not use special characters or emojis or itemized lists. If you don't know the answer to a question, just say that you don't know, or that you haven't got that information yet."
#                     )
#                 },
#                 {
#                     "role": "user", 
#                     "content": question
#                 }
#             ],
#             previous_response_id=self.current_response_id,
#             tools=[
#                 {
#                     "type": "file_search",
#                     "vector_store_ids": [self.vector_store_id],
#                 }
#             ]
#         )

#         self.current_response_id = response.id

#         # Extract the assistant's reply
#         reply = response.output_text
#         # print (f"LLM response:", response)
#         # # print (f"LLM output:", response.output)
#         # print (f"LLM response.output_text:", reply)

#         return reply

class Ollama_demonstration_description:
    
    def __init__(self):

        self.ollama_info_model_creative = LLM_MODEL_NAME
        print("Ollama demonstration initialized.")

        ### CREATING THE EMBEDDINGS FROM COMPETITION DOCUMENTS
        if os.path.exists(COMP_EMBEDDINGS_FILE):
            os.remove(COMP_EMBEDDINGS_FILE)
            print(f"Removed old embeddings file: {COMP_EMBEDDINGS_FILE}")

        print("Loading documents...")
        documents_competition = self.load_documents(mode="competition")
        if not documents_competition:
            print("No .txt or .json files found in the competition documents folder.")
            return
        print(f"Found {len(documents_competition)} competition documents.")

        chunks = self.create_chunks(documents_competition, mode="competition")
        print(f"Created {len(chunks)} chunks.")

        embeddings = self.create_embeddings(chunks)
        self.save_embeddings(chunks, embeddings, mode="competition")
        print("Embeddings for competition created and saved.")

        ### CREATING THE EMBEDDINGS FROM DEMONSTRATION DOCUMENTS
        if os.path.exists(DEMO_EMBEDDINGS_FILE):
            os.remove(DEMO_EMBEDDINGS_FILE)
            print(f"Removed old embeddings file: {DEMO_EMBEDDINGS_FILE}")

        print("Loading documents...")
        documents_demonstration = self.load_documents(mode="demonstration")
        if not documents_demonstration:
            print("No .txt or .json files found in the demonstration documents folder.")
            return
        print(f"Found {len(documents_demonstration)} demonstration documents.")

        chunks = self.create_chunks(documents_demonstration, mode="demonstration")
        print(f"Created {len(chunks)} chunks.")

        embeddings = self.create_embeddings(chunks)
        self.save_embeddings(chunks, embeddings, mode="demonstration")
        print("Embeddings for demonstration created and saved.")

        print("RAG is ready.")

    def load_documents(self, mode):

        documents = []
        if mode == "demonstration":
            documentation_dir = DEMO_DOCUMENTATION_DIR
        else: # mode == "competition"
            documentation_dir = COMP_DOCUMENTATION_DIR

        for filename in os.listdir(documentation_dir):
            path_txt = os.path.join(documentation_dir, filename)

            if not os.path.isfile(path_txt):
                continue

            if not filename.lower().endswith(".txt"):
                continue

            with open(path_txt, "r", encoding="utf-8") as f:
                text = f.read()

            documents.append({
                "filename": filename,
                "text": text
            })

        for filename in os.listdir(CONFIG_FILES_DIR):
            path_json = os.path.join(CONFIG_FILES_DIR, filename)

            if not os.path.isfile(path_json):
                continue

            if not filename.lower().endswith(".json"):
                continue

            with open(path_json, "r", encoding="utf-8") as f:
                data = json.load(f)

            text = json.dumps(data, ensure_ascii=False, indent=2)


            documents.append({
                "filename": filename,
                "text": text
            })

        return documents
        
    def create_chunks(self, documents, mode):

        chunks = []
        if mode == "demonstration":
            chunk_size = DEMO_CHUNK_SIZE
        else: # mode == "competition"
            chunk_size = COMP_CHUNK_SIZE

        for document in documents:
            text = document["text"]
            words = text.split()

            for i in range(0, len(text), chunk_size):
                chunk = " ".join(words[i:i + chunk_size])

                if chunk.strip():
                    chunks.append({
                        "filename": document["filename"],
                        "text": chunk
                    })

        return chunks   
        
    def create_embeddings(self, chunks):
            embeddings = []
    
            print(f"Creating embeddings for {len(chunks)} chunks...")
    
            for i, chunk in enumerate(chunks):
                print(f"Embedding {i + 1}/{len(chunks)}: ")
    
                response = ollama.embed(
                    model=EMBEDDINGS_MODEL_NAME,
                    input=chunk["text"]
                )
                embeddings.append(response["embeddings"][0])
    
            return np.array(embeddings, dtype=np.float32)

    def save_embeddings(self,chunks, embeddings, mode):

        texts =np.array(
            [chunk["text"] for chunk in chunks],
            dtype=object
        )

        filenames = np.array(
            [chunk["filename"] for chunk in chunks],
            dtype=object
        )   


        if mode == "demonstration":
            EMBEDDINGS_FILE = DEMO_EMBEDDINGS_FILE
        else: # mode == "competition"
            EMBEDDINGS_FILE = COMP_EMBEDDINGS_FILE

        np.savez(
            EMBEDDINGS_FILE,
            embeddings=embeddings,
            texts=texts,
            filenames=filenames
        )

        print(f"Embeddings saved to {EMBEDDINGS_FILE}")
    
    def load_embeddings(self, mode):
        
        if mode == "demonstration":
            embeddings_file = DEMO_EMBEDDINGS_FILE
        else: # mode == "competition"
            embeddings_file = COMP_EMBEDDINGS_FILE

        data = np.load(
            embeddings_file,
            allow_pickle=True
        )

        return (data["embeddings"], data["texts"], data["filenames"])

    def cosine_similarity(self, a, b):
            return np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b))

    def search(self, question, embeddings, texts, filenames, mode):
        
        if mode == "demonstration":
            top_k = DEMO_TOP_K
        else: # mode == "competition"
            top_k = COMP_TOP_K

        response = ollama.embed(
            model=EMBEDDINGS_MODEL_NAME,
            input=question
        )

        question_embedding = np.array(response["embeddings"][0], dtype=np.float32)

        scores = []

        for i, embedding in enumerate(embeddings):
            score = self.cosine_similarity(question_embedding, embedding)
            scores.append((score, i))

        scores.sort(reverse=True)

        results = []

        for score, index in scores[:top_k]:
            results.append({
                "score": score,
                "text": texts[index],
                "filename": filenames[index]
            })

        return results

    def answer_demo_question(self, question, results):

        context = "\n\n".join(
            f"[Source: {r['filename']}]\n{r['text']}"
            for r in results
        )

        prompt = f"""

            Your name is CHARMIE and you are a robot assistant. 
            Answer the questions using the provided documents.
            If the answer cannot be found within the documents, 
            say that you don't know how to answer that question
            because you don't have enough information.

            Don't mention the files (they are like the robot's brain and internal knowledge)

            When giving the response keep in mind that it will be useed a TTS converter. 
            Keep you responses natural and without symbols, sounds or side notes.
            Always talk about yourself in the first person.

            Documents:
            {context}

            USER QUESTION:
            {question}

            """

        response = ollama.chat(
            model=LLM_MODEL_NAME,
            messages= [
                {"role": "user",
                "content": prompt}
            ]
        )

        return response["message"]["content"]

    def answer_competition_question(self, question, results):

        context = "\n\n".join(
            f"[Source: {r['filename']}]\n{r['text']}"
            for r in results
        )

        prompt = f"""

        Your name is CHARMIE and you are a robot assistant. 
        Answer the questions using the provided documents.
        If the answer cannot be found within the documents, 
        say that you don't know how to answer that question
        because you don't have enough information.

        Don't mention the files (they are like the robot's brain and internal knowledge)

        When giving the response keep in mind that it will be useed a TTS converter. 
        Keep you responses natural and without symbols, sounds or side notes.
        Always talk about yourself in the first person.
        Keep your answers short and concise, no need for explanations or introdutory text. 
        Only respond to what you are asked, don't add more info than necessary.
        The file objects classes has all the classes of objects and where to find them in the house
        The file objects has the names of the objects and the respective class.
        The files rooms and furnitures have all the rooms and furnitures available in the house

        Documents:
        {context}

        USER QUESTION:
        {question}

        """

        response = ollama.chat(
            model=LLM_MODEL_NAME,
            messages= [
                {"role": "user",
                "content": prompt}
            ]
        )

        return response["message"]["content"]

    def get_answer(self, question, mode):
        
        embeddings, texts, filenames = self.load_embeddings(mode= mode)
        if mode == "demonstration":
            results = self.search(question, embeddings, texts, filenames, mode="demonstration")
            answer = self.answer_demo_question(question, results)
        else: # mode == "competition"
            results = self.search(question, embeddings, texts, filenames, mode="competition")
            answer = self.answer_competition_question(question, results)
        
        return answer

    








