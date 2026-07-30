FROM arm64v8/ros:humble-ros-base

# Set non-interactive mode for apt
ENV DEBIAN_FRONTEND=noninteractive

# Update and install basic dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    wget \
    curl \
    python3-pip \
    python3-colcon-common-extensions \
    qtbase5-dev \
    qtdeclarative5-dev \
    qml-module-qtquick-controls2 \
    qml-module-qtquick-layouts \
    v4l-utils \
    libcamera-dev \
    libopencv-dev \
    python3-opencv \
    && rm -rf /var/lib/apt/lists/*

# Install specific Python dependencies for the project
COPY requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt || true
RUN pip3 install --no-cache-dir \
    pymodbus \
    pydantic \
    pyserial \
    PyYAML

# Note: HailoRT installation is tricky in Docker. For runtime on Pi 5, 
# it's usually better to use volume mounts for the Hailo PCIe device:
# `docker run --device /dev/hailo0 ...`
# The following assumes HailoRT deb is available or installed manually:
# RUN wget https://hailo-repo.s3.amazonaws.com/hailort/hailort-X.Y.Z-arm64.deb && dpkg -i hailort*.deb

# Create workspace directory
WORKDIR /ros2_ws

# Copy source code (Make sure to add .dockerignore)
COPY src/ /ros2_ws/src/

# Install ROS dependencies using rosdep
RUN apt-get update && rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*

# Build the ROS 2 workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install --executor sequential"

# Source the workspace on container startup
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

# Entrypoint to setup ROS environment
COPY scripts_deploy/entrypoint.sh /
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
