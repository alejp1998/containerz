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

## CLI Wrappers & Bashrc Configuration

Both containers have CLI wrappers in `~/.bashrc`. Add the following to your `~/.bashrc`:

```bash
# ============ Container CLI Wrappers ============

# Ollama wrappers
ollama() {
  docker exec ollama ollama "$@"
}
export -f ollama

# llama-cpp wrappers
llama-cpp-completion() {
  docker exec llama-cpp curl -s http://localhost:8080/v1/completions "$@"
}
llama-cpp-chat() {
  docker exec llama-cpp curl -s http://localhost:8080/v1/chat/completions "$@"
}
export -f llama-cpp-completion llama-cpp-chat

# ROS 2 wrappers
ros2() {
  docker exec ros2-humble-persistent ros2 "$@"
}
ros2-shell() {
  docker exec -it ros2-humble-persistent bash
}
export -f ros2 ros2-shell

# Container shortcuts
docker-compose-up() {
  cd ~/dev/containerz
  docker compose -f ollama/docker-compose.yml -f llama-cpp/docker-compose.yml -f ros-humble/docker-compose.yml up -d
}
docker-compose-down() {
  cd ~/dev/containerz
  docker compose -f ollama/docker-compose.yml -f llama-cpp/docker-compose.yml -f ros-humble/docker-compose.yml down
}
docker-compose-logs() {
  cd ~/dev/containerz
  docker compose -f ollama/docker-compose.yml -f llama-cpp/docker-compose.yml -f ros-humble/docker-compose.yml logs -f "$@"
}
export -f docker-compose-up docker-compose-down docker-compose-logs

# Container status
container-status() {
  echo "=== Docker Containers ==="
  docker ps --format "table {{.Names}}\t{{.Status}}\t{{.Ports}}"
}
export -f container-status

# GPU status for each container
gpu-status() {
  echo "=== GPU Usage (Ollama) ==="
  docker exec ollama nvidia-smi --query-gpu=name,memory.used,memory.total --format=csv,noheader
  echo ""
  echo "=== GPU Usage (llama-cpp) ==="
  docker exec llama-cpp nvidia-smi --query-gpu=name,memory.used,memory.total --format=csv,noheader
  echo ""
  echo "=== GPU Usage (ROS 2) ==="
  docker exec ros2-humble-persistent nvidia-smi --query-gpu=name,memory.used,memory.total --format=csv,noheader
}
export -f gpu-status
```

Then reload your shell:
```bash
source ~/.bashrc
```

### Usage Examples

```bash
# Ollama
ollama list
ollama pull qwen2.5:14b
ollama run qwen2.5:14b "What is containerization?"

# llama-cpp (via REST API)
llama-cpp-chat -X POST -H "Content-Type: application/json" \
  -d '{"model":"gpt-oss-20b","messages":[{"role":"user","content":"Hello"}]}'

# ROS 2
ros2 topic list
ros2 run demo_nodes_cpp talker
ros2-shell  # Interactive shell in ROS container

# Container management
docker-compose-up      # Start all containers
docker-compose-down    # Stop all containers
docker-compose-logs    # View logs (use -f for follow)
container-status       # See all running containers
gpu-status            # Check GPU memory per container
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

## Systemd Services

Run containers automatically on boot and manage them with `systemctl`. Create systemd service files:

### 1. Ollama Service

Create `/etc/systemd/system/ollama-container.service`:

```ini
[Unit]
Description=Ollama Docker Container
After=docker.service
Requires=docker.service
StartLimitIntervalSec=60
StartLimitBurst=3

[Service]
Type=simple
WorkingDirectory=/home/alejp/dev/containerz/ollama
ExecStart=/usr/bin/docker compose up -d ollama
ExecStop=/usr/bin/docker compose down
Restart=on-failure
RestartSec=10
User=alejp
Environment="PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin"

[Install]
WantedBy=multi-user.target
```

### 2. llama-cpp Service

Create `/etc/systemd/system/llama-cpp-container.service`:

```ini
[Unit]
Description=llama.cpp Docker Container
After=docker.service
Requires=docker.service
StartLimitIntervalSec=60
StartLimitBurst=3

[Service]
Type=simple
WorkingDirectory=/home/alejp/dev/containerz/llama-cpp
ExecStart=/usr/bin/docker compose up -d llama-cpp
ExecStop=/usr/bin/docker compose down
Restart=on-failure
RestartSec=10
User=alejp
Environment="PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin"

[Install]
WantedBy=multi-user.target
```

### 3. ROS 2 Service

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
