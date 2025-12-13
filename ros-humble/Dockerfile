FROM osrf/ros:humble-desktop-full

# Set environment variables to avoid interactive prompts
ENV DEBIAN_FRONTEND=noninteractive

# Install CycloneDDS and additional dev tools
RUN apt-get update && apt-get install -y \
    ros-humble-rmw-cyclonedds-cpp \
    python3-colcon-common-extensions \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# Copy CycloneDDS config
COPY cyclonedds.xml /etc/cyclonedds.xml

# Set environment variables globally so they persist for non-interactive shells
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV CYCLONEDDS_URI=file:///etc/cyclonedds.xml
ENV ROS_DOMAIN_ID=0

# Source ROS 2 setup in bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# Set working directory
WORKDIR /workspace

CMD ["/bin/bash"]
