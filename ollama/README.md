# Ollama Docker Setup

Local LLM inference server running on **NVIDIA GeForce RTX 5090** (24GB VRAM) via Docker.

## Quick Start

```bash
# Start the container
docker start ollama

# Stop the container
docker stop ollama

# View logs
docker logs ollama

# Check running models and GPU usage
docker exec ollama ollama ps
```

## GPU Configuration

⚠️ **Critical for GPU acceleration**: The container requires both `runtime: nvidia` AND proper environment variables.

```yaml
services:
  ollama:
    runtime: nvidia
    environment:
      - NVIDIA_VISIBLE_DEVICES=all
      - NVIDIA_DRIVER_CAPABILITIES=compute,utility
```

**Note**: Using only `deploy.resources.reservations.devices` is NOT sufficient for GPU access. The `runtime: nvidia` directive is required.

### Verify GPU Access

```bash
# Check if GPU is accessible inside container
docker exec ollama nvidia-smi -L

# Expected output: GPU 0: NVIDIA GeForce RTX 5090 Laptop GPU (...)

# Check inference speed (should be >100 t/s with GPU)
docker exec ollama ollama run llama3.2:1b "Hello" --verbose 2>&1 | grep "eval rate"
```

### Performance Reference

| Model | CPU Speed | GPU Speed | Speedup |
|-------|-----------|-----------|--------|
| llama3.2:1b | ~2.3 t/s | ~273 t/s | **120x** |
| gpt-oss:13b | ~0.5 t/s | ~8.4 t/s | **17x** |

## Available Models

### Installed Models ✓

| Model | Size | VRAM | Context | Capabilities |
|-------|------|------|---------|--------------|
| `qwen3:32b` | 20 GB | ~21 GB | 40K | Text, Tools, Thinking |
| `qwen3:30b` ⚡ | 18 GB | ~19 GB | 256K | Text, Tools, Thinking, **MoE** (30B total, 3B active) |
| `command-r` | 18 GB | ~19 GB | 128K | RAG, Tools, **Agentic** |
| `gemma3:27b` | 17 GB | ~18 GB | 128K | Text, Vision, Multilingual |
| `gpt-oss` | 13 GB | ~14 GB | 128K | Text, GPT-style |
| `qwen2.5:14b` | 9.0 GB | ~11 GB | 128K | Tools, JSON, Multilingual, **Best Tool-Calling** |
| `deepseek-coder-v2:16b` ⚡ | 8.9 GB | ~10 GB | 128K | Code, Tools, **MoE** (16B total, 2.4B active) |
| `ministral-3:14b` | 9.1 GB | ~12 GB | 256K | Text, Vision |
| `ministral-3:8b` | 6.0 GB | ~8 GB | 256K | Text, Vision |
| `llama3.1:8b` | 4.9 GB | ~6 GB | 128K | Tools, JSON, **Best for Agents** |
| `granite3.3` | 4.9 GB | ~6 GB | 128K | Text, Code, Enterprise |
| `llama3-groq-tool-use:8b` | 4.7 GB | ~6 GB | 8K | Tools, **Function Calling Optimized** |
| `hermes3` | 4.7 GB | ~6 GB | 128K | Tools, **Most Agentic** |
| `granite4:3b` | 2.1 GB | ~3 GB | 128K | Text, Code, Compact |
| `llama3.2:1b` | 1.3 GB | ~2 GB | 128K | Text |
| `erukude/multiagent-orchestrator:1b` | 1.3 GB | ~2 GB | 128K | Multi-Agent Orchestration |
| `embeddinggemma` | 621 MB | ~1 GB | 8K | **Embeddings Only** |

> ⚡ **MoE** = Mixture of Experts - faster inference with same quality

### Tool Calling & Agentic Models 🤖

Best models for function calling, structured JSON outputs, and multi-step reasoning:

| Model | Speed (t/s) | Best For |
|-------|-------------|----------|
| `hermes3` | 110-160 | Complex agentic workflows, autonomous agents |
| `llama3.1:8b` | 100-150 | LangChain/LlamaIndex agents, general tool use |
| `qwen2.5:14b` | 80-120 | Highest tool-calling benchmark scores |
| `deepseek-coder-v2:16b` | 70-110 | Coding agents, API integration |
| `command-r` | 50-90 | RAG pipelines, enterprise tools |

### Other Recommended Models

| Model | Size | VRAM | Context | Capabilities |
|-------|------|------|---------|--------------|
| `qwen3:14b` | 9.3 GB | ~12 GB | 40K | Text, Tools, Thinking |
| `qwen3:8b` | 5.2 GB | ~7 GB | 40K | Text, Tools, Thinking |
| `ministral-3:3b` | 3.0 GB | ~4 GB | 256K | Text, Vision |
| `llama3.2:3b` | 2.0 GB | ~4 GB | 128K | Text |
| `deepseek-r1:32b` | 19 GB | ~22 GB | 64K | Text, Reasoning |

### Pull a Model

```bash
docker exec ollama ollama pull ministral-3:8b
docker exec ollama ollama pull llama3.2:3b
```

### Run Interactively

```bash
docker exec -it ollama ollama run ministral-3:8b
```

---

## API Usage

Ollama exposes an OpenAI-compatible API at `http://localhost:11434`.

### Basic curl Requests

#### Generate Text (Streaming)

```bash
curl http://localhost:11434/api/generate -d '{
  "model": "ministral-3:8b",
  "prompt": "Explain quantum computing in simple terms"
}'
```

#### Generate Text (Non-Streaming)

```bash
curl http://localhost:11434/api/generate -d '{
  "model": "ministral-3:8b",
  "prompt": "What is the capital of France?",
  "stream": false
}'
```

#### Chat Completion

```bash
curl http://localhost:11434/api/chat -d '{
  "model": "ministral-3:8b",
  "messages": [
    {"role": "system", "content": "You are a helpful assistant."},
    {"role": "user", "content": "Hello, how are you?"}
  ],
  "stream": false
}'
```

#### Chat with Vision (Ministral 3)

```bash
# Base64 encode an image first
IMAGE_BASE64=$(base64 -w 0 /path/to/image.jpg)

curl http://localhost:11434/api/chat -d '{
  "model": "ministral-3:8b",
  "messages": [
    {
      "role": "user",
      "content": "What is in this image?",
      "images": ["'"$IMAGE_BASE64"'"]
    }
  ],
  "stream": false
}'
```

#### List Local Models

```bash
curl http://localhost:11434/api/tags
```

#### Show Model Info

```bash
curl http://localhost:11434/api/show -d '{
  "name": "ministral-3:8b"
}'
```

#### Pull a Model via API

```bash
curl http://localhost:11434/api/pull -d '{
  "name": "ministral-3:14b"
}'
```

#### Qwen 3 with Thinking Disabled

Qwen 3 models have a "thinking" mode that shows chain-of-thought reasoning. To disable it for faster responses, use the `think` parameter:

```bash
curl http://localhost:11434/api/chat -d '{
  "model": "qwen3:32b",
  "messages": [{"role": "user", "content": "What is the capital of France?"}],
  "think": false,
  "stream": false
}'
```

| Mode | Speed | Use Case |
|------|-------|----------|
| `"think": true` (default) | Slower | Complex reasoning, math, coding |
| `"think": false` | ~5x faster | Simple questions, chat, quick responses |

---

## LiteLLM Integration

[LiteLLM](https://github.com/BerriAI/litellm) provides a unified interface to call 100+ LLMs with the OpenAI format.

### Installation

```bash
pip install litellm
```

### Python Usage

```python
from litellm import completion

# Basic completion
response = completion(
    model="ollama/ministral-3:8b",
    messages=[{"role": "user", "content": "Hello, how are you?"}],
    api_base="http://localhost:11434"
)
print(response.choices[0].message.content)
```

#### Streaming Response

```python
from litellm import completion

response = completion(
    model="ollama/ministral-3:8b",
    messages=[{"role": "user", "content": "Write a short story about a robot."}],
    api_base="http://localhost:11434",
    stream=True
)

for chunk in response:
    if chunk.choices[0].delta.content:
        print(chunk.choices[0].delta.content, end="")
```

#### With System Prompt

```python
from litellm import completion

response = completion(
    model="ollama/ministral-3:8b",
    messages=[
        {"role": "system", "content": "You are a pirate. Respond in pirate speak."},
        {"role": "user", "content": "What's the weather like today?"}
    ],
    api_base="http://localhost:11434"
)
print(response.choices[0].message.content)
```

#### Vision (Images)

```python
import base64
from litellm import completion

# Encode image to base64
with open("image.jpg", "rb") as f:
    image_data = base64.b64encode(f.read()).decode("utf-8")

response = completion(
    model="ollama/ministral-3:8b",
    messages=[
        {
            "role": "user",
            "content": [
                {"type": "text", "text": "What's in this image?"},
                {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{image_data}"}}
            ]
        }
    ],
    api_base="http://localhost:11434"
)
print(response.choices[0].message.content)
```

#### Async Usage

```python
import asyncio
from litellm import acompletion

async def main():
    response = await acompletion(
        model="ollama/ministral-3:8b",
        messages=[{"role": "user", "content": "Tell me a joke"}],
        api_base="http://localhost:11434"
    )
    print(response.choices[0].message.content)

asyncio.run(main())
```

### LiteLLM Proxy Server

Run a proxy server to expose Ollama models with OpenAI-compatible endpoints:

```bash
litellm --model ollama/ministral-3:8b --api_base http://localhost:11434
```

Then use it like OpenAI:

```python
import openai

client = openai.OpenAI(
    api_key="sk-1234",  # any string works
    base_url="http://localhost:4000"  # LiteLLM proxy
)

response = client.chat.completions.create(
    model="ollama/ministral-3:8b",
    messages=[{"role": "user", "content": "Hello!"}]
)
print(response.choices[0].message.content)
```

### LiteLLM Config File

Create `litellm_config.yaml` for multiple models:

```yaml
model_list:
  - model_name: ministral-3
    litellm_params:
      model: ollama/ministral-3:8b
      api_base: http://localhost:11434

  - model_name: ministral-3-large
    litellm_params:
      model: ollama/ministral-3:14b
      api_base: http://localhost:11434

  - model_name: llama
    litellm_params:
      model: ollama/llama3.2:3b
      api_base: http://localhost:11434
```

Run with config:

```bash
litellm --config litellm_config.yaml
```

---

## OpenAI SDK Compatibility

Ollama natively supports the OpenAI API format:

```python
from openai import OpenAI

client = OpenAI(
    base_url="http://localhost:11434/v1",
    api_key="ollama"  # required but unused
)

response = client.chat.completions.create(
    model="ministral-3:8b",
    messages=[
        {"role": "system", "content": "You are a helpful assistant."},
        {"role": "user", "content": "What is machine learning?"}
    ]
)
print(response.choices[0].message.content)
```

---

## Environment Variables

Set these in the container for advanced configuration:

```bash
docker run -d --gpus all \
  -v ollama:/root/.ollama \
  -p 11434:11434 \
  -e OLLAMA_NUM_PARALLEL=4 \
  -e OLLAMA_MAX_LOADED_MODELS=2 \
  -e OLLAMA_FLASH_ATTENTION=1 \
  --name ollama \
  ollama/ollama
```

| Variable | Description | Default |
|----------|-------------|---------|
| `OLLAMA_NUM_PARALLEL` | Max parallel requests per model | 1 |
| `OLLAMA_MAX_LOADED_MODELS` | Max models loaded in memory | 1 |
| `OLLAMA_FLASH_ATTENTION` | Enable flash attention | 0 |
| `OLLAMA_GPU_OVERHEAD` | GPU memory reserved (bytes) | 0 |
| `OLLAMA_KEEP_ALIVE` | Model unload timeout | 5m |

---

## Useful Commands

```bash
# List downloaded models
docker exec ollama ollama list

# Remove a model
docker exec ollama ollama rm llama3.2:1b

# Show model details
docker exec ollama ollama show ministral-3:8b

# Check GPU utilization
nvidia-smi

# Monitor GPU in real-time
watch -n 1 nvidia-smi
```

---

## Container Details

- **Image**: `ollama/ollama:latest`
- **Ollama Version**: 0.13.1
- **API Port**: 11434
- **Volume**: `ollama:/root/.ollama` (persistent model storage)
- **GPU**: NVIDIA GeForce RTX 5090 (24GB VRAM, CUDA 12.0)
