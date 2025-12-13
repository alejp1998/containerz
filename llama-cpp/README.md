# llama.cpp Container

High-performance inference server using the official [llama.cpp Docker image](https://github.com/ggml-org/llama.cpp/blob/master/docs/docker.md) with native CUDA acceleration.

## Overview

This setup uses the **official `ghcr.io/ggml-org/llama.cpp:server-cuda` image**, which provides:

- **OpenAI-compatible API** for seamless integration
- **Native CUDA support** with automatic GPU detection  
- **Direct GGUF model loading** without conversion
- **Flash attention** and other optimizations enabled
- **Automatic architecture detection** (works with all modern GPUs including RTX 5090)

For detailed information, see the [official llama.cpp Docker documentation](https://github.com/ggml-org/llama.cpp/blob/master/docs/docker.md).

## Quick Start

```bash
# Start the server
docker compose up -d

# Check health
curl http://localhost:8080/health

# List available models  
curl http://localhost:8080/v1/models
```

## Changing Models (Simple)

**Option 1: Environment variable (Simplest - Recommended)**

```bash
# Set the MODEL_PATH environment variable
export MODEL_PATH="Ministral-3B-instruct-2501-Q4_K_M.gguf"
docker compose down && docker compose up -d
```

Or directly in one command:

```bash
MODEL_PATH="Ministral-3B-instruct-2501-Q4_K_M.gguf" docker compose up -d
```

The default model is set in `docker-compose.yml`:
```yaml
environment:
  MODEL_PATH: gpt-oss-20b-Q4_K_M.gguf
```

**Option 2: Edit docker-compose.yml (For permanent changes)**

Edit [docker-compose.yml](docker-compose.yml) and change the `MODEL_PATH` environment variable:

```yaml
environment:
  MODEL_PATH: your-model-Q4_K_M.gguf
```

Then restart:

```bash
docker compose down && docker compose up -d
```

## Model Selection: Single vs Multiple

⚠️ **Important difference from Ollama:**

- **llama.cpp**: Loads **one model at startup** into GPU memory (fast, dedicated resources)
- **Ollama**: Can load/switch between multiple models (flexible, memory-efficient)

llama.cpp is optimized for **single-model high-performance inference**. To use a different model, you must restart the container. There is **no API to dynamically switch models** like in Ollama.

### Use llama.cpp when:
- You want maximum performance with one model
- You need OpenAI-compatible API
- Memory bandwidth is a priority
- You're benchmarking/profiling

### Use Ollama when:
- You want to easily switch between many models
- You want built-in model discovery and management
- Memory efficiency matters
- You need a simple, self-contained solution

## Configuration

For advanced tuning, edit the full command in [docker-compose.yml](docker-compose.yml):

```yaml
command: >
  -m /models/your-model-Q4_K_M.gguf
  --host 0.0.0.0
  --port 8080
  -c 16384              # Context window
  -ngl 99               # GPU layers (99 = all layers)
  --temp 0              # Temperature (0 = deterministic)
  --flash-attn on       # Enable flash attention
  --top-k 40            # Top-k sampling
  --top-p 0.95          # Top-p sampling
```

## Finding and Downloading Models

GGUF models are available from Hugging Face. Look for repositories with `-GGUF` in the name:

- **[bartowski](https://huggingface.co/bartowski)** - Most popular, well-quantized (recommended)
- **[MaziyarPanahi](https://huggingface.co/MaziyarPanahi)** - Alternative quantizations
- **[lmstudio-community](https://huggingface.co/lmstudio-community)** - LM Studio optimized models
- **[unsloth](https://huggingface.co/unsloth)** - Specialized variants

### Example Downloads

```bash
cd ./models

# GPT-OSS-20b (Fast, high quality)
wget https://huggingface.co/unsloth/gpt-oss-20b-GGUF/resolve/main/gpt-oss-20b-Q4_K_M.gguf

# Ministral-3B (Small, very fast)
wget https://huggingface.co/bartowski/Ministral-3B-instruct-2501-GGUF/resolve/main/Ministral-3B-instruct-2501-Q4_K_M.gguf

# Qwen2.5-14B (Better reasoning)
wget https://huggingface.co/bartowski/Qwen2.5-14B-Instruct-GGUF/resolve/main/Qwen2.5-14B-Instruct-Q4_K_M.gguf

# Llama 3.1-8B (General purpose)
wget https://huggingface.co/bartowski/Meta-Llama-3.1-8B-Instruct-GGUF/resolve/main/Meta-Llama-3.1-8B-Instruct-Q4_K_M.gguf

# Granite-3.1-3B (Very compact)
wget https://huggingface.co/bartowski/granite-3.1-3b-a800m-instruct-GGUF/resolve/main/granite-3.1-3b-a800m-instruct-Q4_K_M.gguf
```

### Quantization Formats

Choose based on your needs (smaller = faster, larger = better quality):

| Format | Size | Speed | Quality | Use Case |
|--------|------|-------|---------|----------|
| Q2_K | ~25% | Very fast | Lower | Mobile/IoT |
| Q3_K_M | ~30% | Fast | Good | Resource constrained |
| **Q4_K_M** | ~35% | **Balanced** | **Very good** | **Recommended** |
| Q5_K_M | ~45% | Slower | Excellent | Quality focused |
| Q6_K | ~55% | Slow | Near-original | Quality critical |

## API Usage

### OpenAI-Compatible Chat API

```bash
# Simple chat completion
curl -X POST http://localhost:8080/v1/chat/completions \
  -H "Content-Type: application/json" \
  -d '{
    "model": "gpt-oss-20b-Q4_K_M.gguf",
    "messages": [
      {"role": "user", "content": "What is machine learning?"}
    ],
    "max_tokens": 256,
    "temperature": 0.7
  }'
```

### Model Management

```bash
# List available models
curl http://localhost:8080/v1/models

# Health check
curl http://localhost:8080/health
```

## Performance Tuning

### GPU Layer Offloading

- `-ngl 99` - Offload all layers to GPU (fastest, needs ~80% of model size in VRAM)
- `-ngl 50` - Offload ~50 layers (hybrid CPU/GPU)
- `-ngl 0` - CPU only (slow, but uses no VRAM)

### Context Window

- `-c 16384` - Good balance for most tasks
- `-c 8192` - Lower memory usage
- `-c 32768` - Maximum context (needs more VRAM)

### Batch Settings

For concurrent requests, adjust:

```yaml
-ub 512     # Batch size for embeddings
-b 512      # Prompt batch size
```

## Notes

### Why Official Image?

The official `ghcr.io/ggml-org/llama.cpp:server-cuda` image:
- ✅ Automatically supports new GPUs (sm_90, future architectures)
- ✅ Regularly updated with latest llama.cpp features
- ✅ Pre-built with all CUDA optimizations
- ✅ Proven and used by thousands

### Ollama Model Incompatibility

Ollama models stored at `/data/ollama/models/blobs/sha256-*` are **not compatible** with standard llama.cpp due to tensor structure differences. Always download GGUF files from Hugging Face.
