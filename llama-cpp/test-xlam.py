import requests
import json

# Configuration
API_URL = "http://localhost:8080/v1/chat/completions"
MODEL = "Llama-xLAM-2-8b-fc-r-Q4_K_M.gguf"

# Define the tools (functions) available to the model
tools = [
    {
        "type": "function",
        "function": {
            "name": "get_current_weather",
            "description": "Get the current weather in a given location",
            "parameters": {
                "type": "object",
                "properties": {
                    "location": {
                        "type": "string",
                        "description": "The city and state, e.g. San Francisco, CA",
                    },
                    "unit": {"type": "string", "enum": ["celsius", "fahrenheit"]},
                },
                "required": ["location"],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "get_stock_price",
            "description": "Get the current stock price for a given symbol",
            "parameters": {
                "type": "object",
                "properties": {
                    "symbol": {
                        "type": "string",
                        "description": "The stock symbol, e.g. AAPL",
                    }
                },
                "required": ["symbol"],
            },
        },
    }
]

# The user query
messages = [
    {"role": "system", "content": "You are a helpful assistant with access to tools."},
    {"role": "user", "content": "What is the weather in London and what is the stock price of Apple?"}
]

# Payload
payload = {
    "model": MODEL,
    "messages": messages,
    "tools": tools,
    "tool_choice": "auto"
}

print(f"Sending request to {API_URL}...")
try:
    response = requests.post(API_URL, json=payload)
    response.raise_for_status()
    result = response.json()
    
    print("\nResponse:")
    print(json.dumps(result, indent=2))
    
    # Check if tools were called
    choice = result['choices'][0]
    if choice['message'].get('tool_calls'):
        print("\nTool Calls Generated:")
        for tool_call in choice['message']['tool_calls']:
            print(f"- Function: {tool_call['function']['name']}")
            print(f"  Arguments: {tool_call['function']['arguments']}")
    else:
        print("\nNo tool calls generated.")
        print("Content:", choice['message']['content'])

except Exception as e:
    print(f"Error: {e}")
