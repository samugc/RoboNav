FROM osrf/ros:humble-desktop
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip python3-colcon-common-extensions python3-numpy \
    python3-matplotlib python3-pil && rm -rf /var/lib/apt/lists/*
RUN pip3 install --no-cache-dir coppeliasim-zmqremoteapi-client
WORKDIR /ros2_ws
COPY src/ src/
RUN . /opt/ros/humble/setup.sh && colcon build
CMD ["bash"]
