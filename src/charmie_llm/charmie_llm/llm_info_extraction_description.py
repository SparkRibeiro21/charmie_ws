import ollama
from openai import OpenAI

import sys
from pathlib import Path

import json

class LLM_info_extraction_description:

    # Initializinng the LLM (loads API key, creates client)
    def __init__(self):

        self.openai_model= "gpt-4o-mini-2024-07-18"

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


    def extract_info(self, request: str, info_type: str):

        info_extractor = [
            {
                "type": "function",
                "function": {
                    "name": "extract_info",
                    "description": f"Extract the {info_type} from the sentence",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            info_type: {
                                "type": "string",
                                "description": f"The {info_type} extracted from the sentence"
                            }
                        },
                        "required": [] 
                    }
                }
            }
        ]

        response = self.client.chat.completions.create(
            model=self.openai_model,
            messages=[{"role": "user", "content": request}],
            tools=info_extractor,
            tool_choice={"type": "function", "function": {"name": "extract_info"}}
        )

        args = response.choices[0].message.tool_calls[0].function.arguments
        parsed = json.loads(args)

        return parsed.get(info_type) or ""   
    
class Ollama_info_extraction_description:

    def __init__(self):
        
        self.ollama_info_model_struct = "gemma3:1b"

        response = ollama.chat(
            model= self.ollama_info_model_struct,
            messages=[{"role":"user",
                       "content":" Your task is to extract info from the commands you are given. The command you'll receive are a transcription of a spoken command and they may contain some errors or misspels. The info you will need to extract will always be a common english word. When you receive the command, you will be also given the type of info you need to extract. Return ONLY the extracted info. Your output should only be one word or phrase, not a sentence. If you cannot find the info in the command, return an empty string."}]
        )

        print(response["message"]["content"])

        self.ollama_info_model_creative = "llama3.2:1b"

        response = ollama.chat(
            model= self.ollama_info_model_creative,
            messages=[{"role":"user",
                       "content":" Your task is to extract info from the commands you are given. The command you'll receive are a transcription of a spoken command and they may contain some errors or misspels. The info you will need to extract will always be a common english word. When you receive the command, you will be also given the type of info you need to extract. Return ONLY the extracted info. Your output should only be one word or phrase, not a sentence. If you cannot find the info in the command, return an empty string."}]
        )

        print(response["message"]["content"])


        print("Ollama info model initialized.")

    def extract_info(self, request: str, info_type: str):

        # pseudo normalization of the command
        normalized_command = ollama.chat(
            model=self.ollama_info_model_creative,
            messages=[
                {
                    "role": "system",
                    "content": (
                        "You correct speech recognition transcription.\n"
                        "Fix spelling and phonetic errors.\n"
                        "Return ONLY the corrected sentence."
                    )
                },
                {
                    "role": "user",
                    "content": request
                }
            ]
        )

        # extract info 
        extraction = ollama.chat(
            model=self.ollama_info_model_struct,
            messages=[
                {
                    "role": "system",
                    "content": (
                        "You are an information extraction engine.\n"
                        "Extract ONLY the requested information.\n"
                        "Return one word or short phrase.\n"
                        "No explanations."
                    )
                },
                {
                    "role": "user",
                    "content": f"""
                        Command: "{normalized_command["message"]["content"]}"
                        Information to extract: {info_type}

                        Answer:
                        """
                }
            ],
            options={"temperature": 0.0}
        )

        return extraction["message"]["content"]