# Containerz

Docker container setups for development workloads with GPU support.

## Containers

| Container | Purpose | Port | Data Location | Link |
|-----------|---------|------|---------------|------|
| [ollama](./ollama/) | LLM inference server | 11434 | `/data/ollama` | [README](./ollama/README.md) |
| [llama-cpp](./llama-cpp/) | High-perf inference server (OpenAI API) | 8080 | `./models/` | [README](./llama-cpp/README.md) |
| [ros-humble](./ros-humble/) | ROS 2 Humble robotics | Host network | `~` mounted | [README](./ros-humble/README.md) |

## Quick Status

```bash
docker ps --format "table {{.Names}}\t{{.Status}}\t{{.Ports}}"
```

## Container Comparison

### Ollama vs llama-cpp

| Feature | Ollama | llama-cpp |
|---------|--------|-----------|
| **Model Format** | Native GGUF + Ollama custom | Standard GGUF only |
| **API** | Custom protocol | OpenAI-compatible |
| **Speed** | Good | Excellent |
| **Model Discovery** | Built-in hub | Hugging Face |
| **Configuration** | Limited | Full control |

**Use Case**:
- **Ollama**: Quick setup, many models available, less configuration
- **llama-cpp**: Performance-critical, need OpenAI API compatibility, model flexibility

## CLI Wrappers

Both containers have CLI wrappers in `~/.bashrc`:

```bash
# Ollama - runs in container
ollama list
ollama pull llama3.2
ollama run qwen3:32b

# ROS 2 - runs in container  
ros2 topic list
ros2 run demo_nodes_cpp talker
ros2-shell  # interactive
```

## GPU Verification

```bash
# Ollama
docker exec ollama nvidia-smi

# llama-cpp
docker exec llama-cpp nvidia-smi

# ROS 2
docker exec ros2-humble-persistent nvidia-smi
```

## Changing Models

### In llama-cpp

Edit [llama-cpp/docker-compose.yml](./llama-cpp/docker-compose.yml):

```yaml
command: >
  -m /models/your-model-Q4_K_M.gguf  # Change this line
  --host 0.0.0.0
  --port 8080
  -ngl 99  # Offload all layers to GPU
```

Then restart:
```bash
cd llama-cpp
docker compose down && docker compose up -d
```

**Find models**: [bartowski](https://huggingface.co/bartowski?search=GGUF) on Hugging Face

**Recommended models**:
- **Fast**: Ministral-3B, Granite-3.1-3B
- **Balanced**: Qwen2.5-14B, Llama-3.1-8B
- **Powerful**: GPT-OSS-20b, Mixtral-8x7B

### In Ollama

```bash
# List available models
ollama list

# Pull a model
ollama pull qwen2.5:14b

# Modify default Ollama models via [ollama/docker-compose.yml](./ollama/docker-compose.yml)
```

## Quick Links

- **[llama.cpp Official Docs](https://github.com/ggml-org/llama.cpp/blob/master/docs/docker.md)** - Complete reference
- **[Hugging Face GGUF Models](https://huggingface.co/models?library=gguf)** - Model repository
- **[bartowski's Models](https://huggingface.co/bartowski)** - Most popular quantizations
