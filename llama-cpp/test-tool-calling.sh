#!/bin/bash

echo "Testing xLAM-2-8b-fc-r function calling..."
echo ""

curl http://localhost:11435/v1/chat/completions \
  -H "Content-Type: application/json" \
  -d '{
    "model": "xlam",
    "messages": [
      {
        "role": "system",
        "content": "You have access to the following tools. Respond with JSON array of tool calls."
      },
      {
        "role": "user",
        "content": "What is the weather in Copenhagen and Stockholm?"
      }
    ],
    "tools": [
      {
        "type": "function",
        "function": {
          "name": "get_weather",
          "description": "Get current weather for a location",
          "parameters": {
            "type": "object",
            "properties": {
              "location": {
                "type": "string",
                "description": "City name"
              },
              "unit": {
                "type": "string",
                "enum": ["celsius", "fahrenheit"],
                "default": "celsius"
              }
            },
            "required": ["location"]
          }
        }
      }
    ],
    "tool_choice": "auto",
    "temperature": 0,
    "max_tokens": 256
  }' | jq '.'

echo ""
echo "Testing simple chat (no tools)..."
echo ""

curl http://localhost:11435/v1/chat/completions \
  -H "Content-Type: application/json" \
  -d '{
    "model": "xlam",
    "messages": [{"role": "user", "content": "Explain what xLAM is in one sentence"}],
    "temperature": 0,
    "max_tokens": 100
  }' | jq -r '.choices[0].message.content'
