#!/bin/bash

source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=/opt/ros/humble/share/turtlebot3_gazebo/models:/ros2_ws/worlds
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/usr/lib/x86_64-linux-gnu/gazebo-11/plugins

echo "Starting Gazebo..."
gzserver /ros2_ws/worlds/dynamic_world/world.model --verbose \
    -slibgazebo_ros_init.so -slibgazebo_ros_factory.so &
sleep 8

echo "Starting Gazebo client..."
gzclient &
sleep 3

echo "Starting obstacle avoidance node..."
ros2 run autonomous_robot obstacle_avoidance
