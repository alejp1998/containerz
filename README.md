# Containerz

Docker container setups for development workloads with GPU support.

## Containers

| Container | Purpose | Port | Data Location | Link |
|-----------|---------|------|---------------|------|
| [gpt-oss-20b](./llama-cpp/) | LLM inference (GPT-OSS 20B) | 11435 | `./llama-cpp/models/` | [README](./llama-cpp/README.md) |
| [ministral-14b](./llama-cpp/) | LLM inference (Ministral 14B Vision) | 11436 | `./llama-cpp/models/` | [README](./llama-cpp/README.md) |
| [embeddinggemma](./llama-cpp/) | Embeddings server (300M) | 11437 | `./llama-cpp/models/` | [README](./llama-cpp/README.md) |
| [ros-humble](./ros-humble/) | ROS 2 Humble robotics (NVIDIA GPU + CycloneDDS) | Host network | `~` mounted | [README](./ros-humble/README.md) |
| ~~[ollama](./ollama/)~~ | ~~LLM inference~~ (deactivated) | ~~11434~~ | `/data/ollama` | [README](./ollama/README.md) |

## Quick Status

```bash
docker ps --format "table {{.Names}}\t{{.Status}}\t{{.Ports}}"
```

## llama-cpp Multi-Model Setup

This repo uses **3 parallel llama-cpp servers** instead of Ollama for better performance and control:

| Feature | Previous (Ollama) | Current (llama-cpp) |
|---------|------------------|---------------------|
| **Model Format** | Native GGUF + Ollama custom | Standard GGUF only |
| **API** | Custom protocol | OpenAI-compatible |
| **Speed** | ~8 t/s for 20B | **~141 t/s for 20B (18x faster)** |
| **Parallel Models** | Sequential (load/unload) | **3 models loaded simultaneously** |
| **Configuration** | Limited | Full control (context, temp, layers) |
| **GPU Offload** | Partial/Dynamic | Full (`-ngl 99`) |

**Why llama-cpp?**
- 18x faster inference than Ollama
- Run multiple specialized models (chat, vision, embeddings) simultaneously
- OpenAI-compatible API for easy integration
- Full GPU layer offloading for maximum performance

### GPU Memory Sharing

All three llama-cpp services run simultaneously on RTX 5090 (24GB VRAM):

| Container | Model | VRAM Usage | Speed |
|-----------|-------|------------|-------|
| gpt-oss-20b | gpt-oss-20b-Q4_K_M | ~11.9 GB | ~141 t/s |
| ministral-14b | Ministral-3-14B-Q4_K_M | ~9.4 GB | ~49 t/s |
| embeddinggemma | embeddinggemma-300M-Q8_0 | ~0.7 GB | ~6 embeds/s |
| rhino-detection | YOLOv8 | ~0.3 GB | N/A |
| **Total** | | **~22.3 GB / 24 GB** | |

✅ All models loaded simultaneously with excellent performance.

## CLI Wrappers & Bashrc Configuration

Your `.bashrc` currently has basic wrappers. Here's what's actually configured:

### Current Configuration

**Ollama wrapper** (already in ~/.bashrc):
```bash
ollama() {
    docker exec ollama ollama "$@"
}
```

**ROS 2 aliases** (already in ~/.bashrc):
```bash
alias ros2='ros2-exec ros2'
alias ros2-shell='sudo docker exec -it -e DISPLAY=$DISPLAY -e XAUTHORITY=/home/alejp/.Xauthority ros2-humble-persistent bash -c "source /opt/ros/humble/setup.bash && bash"'
alias rviz2='ros2-exec rviz2'
alias gazebo='ros2-exec gazebo'
# ... and many more
```

### Recommended Additions to ~/.bashrc

Add these enhanced wrappers and utilities to your `~/.bashrc`:

```bash
# ============ llama-cpp REST API wrappers ============
llama-cpp-completion() {
  docker exec llama-cpp curl -s http://localhost:11435/v1/completions "$@"
}

llama-cpp-chat() {
  docker exec llama-cpp curl -s http://localhost:11435/v1/chat/completions "$@"
}

export -f llama-cpp-completion llama-cpp-chat

# ============ Container management shortcuts ============
container-up() {
  cd ~/dev/containerz
  docker compose -f ollama/docker-compose.yml -f llama-cpp/docker-compose.yml -f ros-humble/docker-compose.yml up -d
  echo "✓ All containers started"
}

container-down() {
  cd ~/dev/containerz
  docker compose -f ollama/docker-compose.yml -f llama-cpp/docker-compose.yml -f ros-humble/docker-compose.yml down
  echo "✓ All containers stopped"
}

container-logs() {
  cd ~/dev/containerz
  docker compose -f ollama/docker-compose.yml -f llama-cpp/docker-compose.yml -f ros-humble/docker-compose.yml logs -f "$@"
}

container-status() {
  echo "=== Docker Containers ==="
  docker ps --format "table {{.Names}}\t{{.Status}}\t{{.Ports}}"
}

gpu-status() {
  echo "=== GPU Memory Usage ==="
  echo ""
  echo "Ollama:"
  docker exec ollama nvidia-smi --query-gpu=name,memory.used,memory.total --format=csv,noheader 2>/dev/null || echo "  (not running)"
  echo ""
  echo "llama-cpp:"
  docker exec llama-cpp nvidia-smi --query-gpu=name,memory.used,memory.total --format=csv,noheader 2>/dev/null || echo "  (not running)"
  echo ""
  echo "ROS 2 Humble:"
  docker exec ros2-humble-persistent nvidia-smi --query-gpu=name,memory.used,memory.total --format=csv,noheader 2>/dev/null || echo "  (not running)"
}

export -f container-up container-down container-logs container-status gpu-status
```

Then reload your shell:
```bash
source ~/.bashrc
```

### Usage Examples

```bash
# Start/stop all containers
container-up
container-down

# View logs (use -f for follow, specify service name for single container)
container-logs
container-logs ollama
container-logs -f llama-cpp

# Check container status
container-status
gpu-status

# Ollama
ollama list
ollama pull qwen2.5:14b

# llama-cpp (REST API calls)
llama-cpp-chat -X POST -H "Content-Type: application/json" \
  -d '{"model":"gpt-oss-20b","messages":[{"role":"user","content":"Hello"}]}'

# ROS 2 (various aliases available)
ros2 topic list
ros2 run demo_nodes_cpp talker
ros2-shell  # Interactive shell
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

## Systemd Services (Optional)

**Note**: Systemd services are **not currently configured**. This section provides instructions if you want to auto-start containers on boot.

### Why Use Systemd Services?

Without systemd services, you must manually run:
```bash
cd ~/dev/containerz && docker compose up -d
```

With systemd services, containers start automatically on boot and can be managed with `systemctl`.

### Setup Instructions

Service files are provided in the `systemd/` directory as template units. To install them for your user (replace `your_username` with your actual username, e.g., `alejp`):

```bash
sudo cp ~/dev/containerz/systemd/*.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now ollama-container@$(whoami).service
sudo systemctl enable --now llama-cpp-container@$(whoami).service
sudo systemctl enable --now ros2-humble@$(whoami).service
```

### 1. Ollama Service

(File provided in `systemd/ollama-container@.service`)

### 2. llama-cpp Service

(File provided in `systemd/llama-cpp-container@.service`)

### 3. ROS 2 Service

(File provided in `systemd/ros2-humble@.service`)

Create `/etc/systemd/system/ros2-humble-container.service`:

```ini
[Unit]
Description=ROS 2 Humble Docker Container
After=docker.service
Requires=docker.service
StartLimitIntervalSec=60
StartLimitBurst=3

[Service]
Type=simple
WorkingDirectory=/home/alejp/dev/containerz/ros-humble
ExecStart=/usr/bin/docker compose up -d ros2-humble-persistent
ExecStop=/usr/bin/docker compose down
Restart=on-failure
RestartSec=10
User=alejp
Environment="PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin"

[Install]
WantedBy=multi-user.target
```

### Managing Services

```bash
# Enable all services to start on boot
sudo systemctl daemon-reload
sudo systemctl enable ollama-container.service
sudo systemctl enable llama-cpp-container.service
sudo systemctl enable ros2-humble-container.service

# Start services
sudo systemctl start ollama-container
sudo systemctl start llama-cpp-container
sudo systemctl start ros2-humble-container

# Check status
sudo systemctl status ollama-container
sudo systemctl status llama-cpp-container
sudo systemctl status ros2-humble-container

# View logs
journalctl -u ollama-container -f
journalctl -u llama-cpp-container -f
journalctl -u ros2-humble-container -f

# Stop services
sudo systemctl stop ollama-container
sudo systemctl stop llama-cpp-container
sudo systemctl stop ros2-humble-container

# Disable from autostart
sudo systemctl disable ollama-container.service
sudo systemctl disable llama-cpp-container.service
sudo systemctl disable ros2-humble-container.service

# Restart all services
sudo systemctl restart ollama-container llama-cpp-container ros2-humble-container
```

### One-Time Setup Script

Save this as `setup-systemd-services.sh` in the repository root:

```bash
#!/bin/bash
set -e

REPO_PATH="/home/alejp/dev/containerz"
SERVICES=("ollama-container" "llama-cpp-container" "ros2-humble-container")

echo "Setting up systemd services for containers..."

# Ollama service
echo "[*] Installing ollama-container.service..."
sudo tee /etc/systemd/system/ollama-container.service > /dev/null <<EOF
[Unit]
Description=Ollama Docker Container
After=docker.service
Requires=docker.service
StartLimitIntervalSec=60
StartLimitBurst=3

[Service]
Type=simple
WorkingDirectory=${REPO_PATH}/ollama
ExecStart=/usr/bin/docker compose up -d ollama
ExecStop=/usr/bin/docker compose down
Restart=on-failure
RestartSec=10
User=$(whoami)
Environment="PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin"

[Install]
WantedBy=multi-user.target
EOF

# llama-cpp service
echo "[*] Installing llama-cpp-container.service..."
sudo tee /etc/systemd/system/llama-cpp-container.service > /dev/null <<EOF
[Unit]
Description=llama.cpp Docker Container
After=docker.service
Requires=docker.service
StartLimitIntervalSec=60
StartLimitBurst=3

[Service]
Type=simple
WorkingDirectory=${REPO_PATH}/llama-cpp
ExecStart=/usr/bin/docker compose up -d llama-cpp
ExecStop=/usr/bin/docker compose down
Restart=on-failure
RestartSec=10
User=$(whoami)
Environment="PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin"

[Install]
WantedBy=multi-user.target
EOF

# ROS 2 service
echo "[*] Installing ros2-humble-container.service..."
sudo tee /etc/systemd/system/ros2-humble-container.service > /dev/null <<EOF
[Unit]
Description=ROS 2 Humble Docker Container
After=docker.service
Requires=docker.service
StartLimitIntervalSec=60
StartLimitBurst=3

[Service]
Type=simple
WorkingDirectory=${REPO_PATH}/ros-humble
ExecStart=/usr/bin/docker compose up -d ros2-humble-persistent
ExecStop=/usr/bin/docker compose down
Restart=on-failure
RestartSec=10
User=$(whoami)
Environment="PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin"

[Install]
WantedBy=multi-user.target
EOF

# Reload systemd
echo "[*] Reloading systemd daemon..."
sudo systemctl daemon-reload

# Enable services
echo "[*] Enabling services..."
for service in "${SERVICES[@]}"; do
  sudo systemctl enable "${service}.service"
done

echo "[✓] Systemd services configured successfully!"
echo ""
echo "To start services manually, run:"
echo "  sudo systemctl start ollama-container"
echo "  sudo systemctl start llama-cpp-container"
echo "  sudo systemctl start ros2-humble-container"
echo ""
echo "Services will automatically start on next boot."
```

Run the setup script:
```bash
chmod +x setup-systemd-services.sh
./setup-systemd-services.sh
```

### Troubleshooting Systemd Services

```bash
# Check service status with detailed output
sudo systemctl status ollama-container -l

# View recent service logs
journalctl -u ollama-container -n 50

# View all logs for a service
journalctl -u ollama-container --no-pager

# Test service manually (without systemd)
cd ~/dev/containerz/ollama
docker compose up -d  # Start
docker ps           # Verify
docker compose down # Stop

# Check Docker socket permissions
ls -l /var/run/docker.sock

# Add current user to docker group (if needed)
sudo usermod -aG docker $USER
# Then log out and back in
```

## Changing Models

### In llama-cpp

Edit [llama-cpp/docker-compose.yml](./llama-cpp/docker-compose.yml):

```yaml
command: >
  -m /models/your-model-Q4_K_M.gguf  # Change this line
  --host 0.0.0.0
  --port 11435
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
