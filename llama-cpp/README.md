# llama.cpp Multi-Model Container Setup

High-performance parallel inference using the official [llama.cpp Docker image](https://github.com/ggml-org/llama.cpp/blob/master/docs/docker.md) with native CUDA acceleration.

## Overview

This setup runs **3 specialized llama-cpp servers simultaneously** on RTX 5090:

| Service | Model | Port | Purpose | VRAM | Speed |
|---------|-------|------|---------|------|-------|
| **gpt-oss-20b** | GPT-OSS 20B Q4_K_M | 11435 | General chat | ~11.9 GB | ~141 t/s |
| **ministral-14b** | Ministral-3 14B Q4_K_M | 11436 | Vision + 256K context | ~9.4 GB | ~49 t/s |
| **embeddinggemma** | Embeddinggemma 300M Q8_0 | 11437 | Embeddings | ~0.7 GB | ~6 embeds/s |

**Total VRAM: ~22 GB / 24 GB** ✅

### Features

- **OpenAI-compatible API** for seamless integration
- **Native CUDA support** with automatic GPU detection  
- **Direct GGUF model loading** without conversion
- **Flash attention** and other optimizations enabled
- **18x faster** than Ollama for same models

For detailed information, see the [official llama.cpp Docker documentation](https://github.com/ggml-org/llama.cpp/blob/master/docs/docker.md).

## Quick Start

```bash
# Start all 3 servers
docker compose up -d

# Check health of all services
curl http://localhost:11435/health  # gpt-oss-20b
curl http://localhost:11436/health  # ministral-14b
curl http://localhost:11437/health  # embeddinggemma

# Test GPT-OSS 20B chat
curl -X POST http://localhost:11435/v1/chat/completions \
  -H "Content-Type: application/json" \
  -d '{"model":"gpt-oss","messages":[{"role":"user","content":"Hello!"}],"max_tokens":50}'

# Test Ministral 14B (vision-capable)
curl -X POST http://localhost:11436/v1/chat/completions \
  -H "Content-Type: application/json" \
  -d '{"model":"ministral","messages":[{"role":"user","content":"Explain quantum computing"}],"max_tokens":100}'

# Test embeddings
curl -X POST http://localhost:11437/v1/embeddings \
  -H "Content-Type: application/json" \
  -d '{"model":"embeddinggemma","input":"Hello world"}'
```

## Architecture

This setup uses **one Docker Compose file** with **3 independent llama-cpp services**, each loading a different model into GPU memory at startup.

### Why Multiple Services?

llama-cpp is optimized for **single-model high-performance inference**. Unlike Ollama (which dynamically loads/unloads models), llama-cpp:
- Loads model once at startup
- Keeps model in GPU memory
- Achieves **18x faster inference** than Ollama
- No model switching overhead

### Model Selection

### Model Selection

**Current Models:**
- **gpt-oss-20b-Q4_K_M.gguf** (11 GB) - Fast general chat
- **Ministral-3-14B-Instruct-2512-Q4_K_M.gguf** (7.7 GB) - Vision + long context
- **embeddinggemma-300M-Q8_0.gguf** (314 MB) - Text embeddings

To change a model, edit `docker-compose.yml` and update the `-m` parameter for the desired service.

## Managing Individual Services

```bash
# Start/stop individual services
docker compose up -d gpt-oss-20b
docker compose stop ministral-14b
docker compose restart embeddinggemma

# View logs
docker compose logs -f gpt-oss-20b
docker compose logs -f ministral-14b

# Stop all
docker compose down
```

## Changing Models (Advanced)

Edit [docker-compose.yml](docker-compose.yml) and change the `-m` parameter for the service:

```yaml
services:
  gpt-oss-20b:
    command: >
      -m /models/your-new-model-Q4_K_M.gguf
      --host 0.0.0.0
      --port 8080
      -c 16384
      -ngl 99
      ...
```

Then restart the specific service:

```bash
docker compose up -d gpt-oss-20b
```

## Performance Comparison: llama.cpp vs Ollama

**Same model (gpt-oss-20b Q4_K_M), same GPU (RTX 5090):**

| Server | Speed | GPU Layers | Notes |
|--------|-------|------------|-------|
| **llama.cpp** | **149 t/s** | 25/25 (100%) | Model stays loaded, full GPU offload |
| Ollama | 8.4 t/s | Partial | Dynamic loading, may share GPU |

### Why is llama.cpp Faster?

1. **Full GPU layer offloading** (`-ngl 99`): All transformer layers run on GPU
2. **Model stays loaded**: No load/unload overhead between requests
3. **Dedicated resources**: Optimized for single-model high-throughput
4. **Flash attention**: Enabled by default (`--flash-attn on`)

### GPU Memory Usage

With gpt-oss-20b-Q4_K_M.gguf (11GB file):
- **llama.cpp**: ~11.9 GB VRAM (model + KV cache)
- **Ollama**: ~10.9 GB VRAM (same model)

Both containers can run simultaneously on RTX 5090 (24GB), sharing GPU memory.

## Notes

### Why Official Image?

The official `ghcr.io/ggml-org/llama.cpp:server-cuda` image:
- ✅ Automatically supports new GPUs (sm_90, future architectures)
- ✅ Regularly updated with latest llama.cpp features
- ✅ Pre-built with all CUDA optimizations
- ✅ Proven and used by thousands

### Ollama Model Incompatibility

Ollama models stored at `/data/ollama/models/blobs/sha256-*` are **not compatible** with standard llama.cpp due to tensor structure differences. Always download GGUF files from Hugging Face.
