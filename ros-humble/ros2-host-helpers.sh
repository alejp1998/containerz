#!/bin/bash
# ROS 2 Host Helper Functions
# Source this file in your .bashrc: source ~/dev/containerz/ros-humble/ros2-host-helpers.sh

# Unalias ros2 if it exists to avoid syntax errors
if alias ros2 >/dev/null 2>&1; then
    unalias ros2
fi

# Check if ROS 2 container is running
ros2_container_check() {
    if [ -z "$(cd ~/dev/containerz/ros-humble && docker compose ps -q ros2-shell)" ]; then
        echo "Error: ROS 2 container (ros2-shell) is not running."
        echo "Start it with: cd ~/dev/containerz/ros-humble && docker compose up -d"
        return 1
    fi
    return 0
}

# Execute ros2 commands in the container using docker compose
function ros2() {
    if ros2_container_check; then
        (cd ~/dev/containerz/ros-humble && docker compose exec -it ros2-shell bash -c "source /opt/ros/humble/setup.bash && ros2 $*")
    fi
}

# Execute ros2 bag commands in the container
function ros2bag() {
    if ros2_container_check; then
        (cd ~/dev/containerz/ros-humble && docker compose exec -it ros2-shell bash -c "source /opt/ros/humble/setup.bash && ros2 bag $*")
    fi
}

# Open an interactive bash shell in the ROS 2 container
function ros2_shell() {
    if ros2_container_check; then
        (cd ~/dev/containerz/ros-humble && docker compose exec -it ros2-shell bash -c "source /opt/ros/humble/setup.bash && exec bash")
    fi
}

# Restart the ROS 2 containers
function ros2_restart() {
    (cd ~/dev/containerz/ros-humble && docker compose restart)
}

# Check ROS 2 container status
function ros2_status() {
    (cd ~/dev/containerz/ros-humble && docker compose ps)
}

echo "ROS 2 host helpers loaded. Available commands:"
echo "  ros2          - Run ros2 commands in the container"
echo "  ros2bag       - Run ros2 bag commands in the container"
echo "  ros2_shell    - Open interactive shell in ROS 2 container"
echo "  ros2_restart  - Restart the ROS 2 containers"
echo "  ros2_status   - Check ROS 2 container status"
