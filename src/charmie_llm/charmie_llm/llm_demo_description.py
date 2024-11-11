from openai import OpenAI
class LLM_demo_description:
    def __init__(self):
        # Define the instructions text
        instructions_text = """
        You are a Home Assistant, your name is Charmie, and your task is to talk with people. Your body was made by the Laboratory of Automation and Robotics (LAR), which is a laboratory of the University of Minho. Your principal developer is Tiago Ribeiro. Your main tasks are: Following people, you can help them carry things since you have a manipulable arm. You can serve at tables, making your applicability vast. Or just talking. 

        Try to be pleasant and fun.

        If people talk to you in portuguese, you will speak European Portuguese, so instead of "você," use "tu," and do not use gerunds. If people talk to you in English, you will talk English(UK). People approach you with fear and hesitation, so try to break the ice and speak more at first contact. Always engage more conversation, even if they only say "olá." What you say is converted to voice, so you should write as if you are speaking and do not use special characters or emojis, except for "[]" that you can use for task commands.
        Don't use itemize or lists of data as output because your text will be transformed in Speech (Text-to-Speech). Make your message as short as possible.

        The Team Members are on "Actual Team Members.txt" file.
        The Tasks that you can perform are on "Assistant Tasks.txt" file.
        Your hardware overview is on "Hardware Overview.txt" file.
        Your actual local(spot), language to use and some other configs are on "Actual Config.txt"
        You have two .pdf files that is the Team Description Paper of Charmie, read to know yourself better.
        """
    # Create a vector store caled "Financial Statements"
        self.client = OpenAI(api_key="api_key") #TODO: CHANGE TO SECRETS
        
        vector_store = self.client.beta.vector_stores.create(name="Documentation")
        self.thread = self.client.beta.threads.create()
        # Ready the files for upload to OpenAI
        folder_path="/home/martins/charmie_ws/src/charmie_llm/charmie_llm/Files/"
        file_paths = ["tdp2023.pdf", "tdp2024.pdf","Actual Config.txt","Actual Team Members.txt", "Assistant Tasks.txt","Hardware Overview.txt"]  # List of your PDF files
        file_streams = [open(folder_path+path, "rb") for path in file_paths]

        # Use the upload and poll SDK helper to upload the files, add them to the vector store,
        # and poll the status of the file batch for completion.
        self.client.beta.vector_stores.file_batches.upload_and_poll(
        vector_store_id=self.vector_store.id, files=file_streams
        )
        self.assistant = self.client.beta.assistants.create(
            name="Charmie",
            instructions=instructions_text,
            model="gpt-3.5-turbo-0125",
            tool_resources={"file_search": {"vector_store_ids": [vector_store.id]}},
        )
        # TODO:
        '''-Add Demo functions. Arm, pick, etc.'''

    def run(self,command="What is the first capital of Portugal?"):
        # Send user input as a message in the thread
        self.client.beta.threads.messages.create(
            thread_id=self.thread.id,
            role="user",
            content=command
        )

        # Wait for the assistant's response
        run = self.client.beta.threads.runs.create_and_poll(
            thread_id=self.thread.id,
            assistant_id=self.assistant.id
        )
    
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
            return assistant_res
        # Handle errors if terminal state is "failed"
        else:
            print("ERRO LLM") #TODO: add Log Node
            return "Info to Programmer: LLM not working"
