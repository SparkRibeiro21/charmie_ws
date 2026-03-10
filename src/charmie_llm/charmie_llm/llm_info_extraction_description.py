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