FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
ENV TURTLEBOT3_MODEL=waffle

RUN apt-get update && apt-get install -y \
    ros-humble-turtlebot3 \
    ros-humble-turtlebot3-simulations \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    python3-pip \
    python3-opencv \
    python3-numpy \
    x11-apps \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws
RUN mkdir -p src

COPY src/autonomous_robot src/autonomous_robot
COPY docker/worlds /ros2_ws/worlds
COPY docker/launch_demo.sh /launch_demo.sh
COPY docker/entrypoint.sh /entrypoint.sh

RUN chmod +x /launch_demo.sh /entrypoint.sh

RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install"

ENTRYPOINT ["/entrypoint.sh"]
