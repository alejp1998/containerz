# ROS 2 Humble Docker Setup

GPU-accelerated ROS 2 Humble container with NVIDIA support.

## Quick Start

```bash
# Start container
docker compose up -d

# Enter shell
ros2-shell

# Run ROS commands directly
ros2 topic list
ros2 run demo_nodes_cpp talker
```

## CLI Aliases (configured in ~/.bashrc)

| Command | Description |
|---------|-------------|
| `ros2` | Run ros2 CLI in container |
| `ros2-shell` | Interactive shell in container |
| `colcon` | Run colcon build system |
| `rosdep` | Run rosdep dependency manager |

## Container Details

| Setting | Value |
|---------|-------|
| Image | `ros2-humble` (custom) |
| Container | `ros2-humble-persistent` |
| Network | Host mode |
| GPU | NVIDIA passthrough |
| Home mount | `/home/alejp` |

## GUI Applications

X11 forwarding is configured. Run GUI apps like RViz:

```bash
ros2-shell
rviz2
```

## Exec Script

The `ros2-exec` script at `~/.local/bin/ros2-exec` handles command execution in the container.
