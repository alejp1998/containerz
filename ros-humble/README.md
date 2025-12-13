# ROS 2 Humble Docker Setup

GPU-accelerated ROS 2 Humble container with NVIDIA support and CycloneDDS.

## Quick Start

```bash
# Build and start container
docker compose up -d --build

# Enter shell
ros2-shell

# Run ROS commands directly (via aliases)
ros2 topic list
ros2 run demo_nodes_cpp talker
```

## Systemd Service

The container can be managed as a systemd service for automatic startup:

```bash
# Install the service
sudo cp ../systemd/ros2-humble@.service /etc/systemd/system/
sudo systemctl daemon-reload

# Enable and start (replace 'alejp' with your username)
sudo systemctl enable ros2-humble@alejp.service
sudo systemctl start ros2-humble@alejp.service

# Check status
sudo systemctl status ros2-humble@alejp.service

# View logs
journalctl -u ros2-humble@alejp.service -f
```

## CLI Aliases (configured in ~/.bashrc)

| Command | Description |
|---------|-------------|
| `ros2` | Run ros2 CLI in container |
| `ros2-shell` | Interactive shell in container |
| `colcon` | Run colcon build system |
| `rosdep` | Run rosdep dependency manager |
| `rviz2` | Launch RViz2 |
| `rqt` | Launch rqt tools |

## Container Details

| Setting | Value |
|---------|-------|
| Image | `ros2-humble:latest` (custom build) |
| Base | `osrf/ros:humble-desktop-full` |
| Container | `ros2-humble-persistent` |
| Network | Host mode |
| IPC | Host (for shared memory) |
| DDS | CycloneDDS |
| GPU | NVIDIA passthrough |
| Home mount | `/home/alejp` |

## DDS Configuration

Uses CycloneDDS with configuration from `cyclonedds.xml`. Environment variables:
- `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
- `CYCLONEDDS_URI=file:///etc/cyclonedds.xml`
- `ROS_DOMAIN_ID=0`

## GUI Applications

X11 forwarding is configured. Run GUI apps like RViz:

```bash
ros2-shell
rviz2
```

## Healthcheck

The container includes a healthcheck that verifies ROS 2 is operational:
- Runs `ros2 topic list` every 30 seconds
- 10 second start period for initialization

## Exec Script

The `ros2-exec` script at `~/.local/bin/ros2-exec` handles command execution in the container.
