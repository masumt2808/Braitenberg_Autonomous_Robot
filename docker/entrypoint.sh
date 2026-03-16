#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=/opt/ros/humble/share/turtlebot3_gazebo/models:/ros2_ws/worlds
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/usr/lib/x86_64-linux-gnu/gazebo-11/plugins

echo "================================================"
echo " Braitenberg Robot - ROS2 Humble + TurtleBot3"
echo "================================================"
echo ""
echo "To launch the dynamic world:"
echo "  gzserver /ros2_ws/worlds/dynamic_world/world.model --verbose -slibgazebo_ros_init.so -slibgazebo_ros_factory.so &"
echo "  sleep 5 && gzclient &"
echo "  ros2 launch turtlebot3_gazebo turtlebot3_state_publisher.launch.py &"
echo "  ros2 run autonomous_robot obstacle_avoidance"
echo ""
echo "For teleop:"
echo "  ros2 run autonomous_robot teleop_node"
echo "================================================"

exec "$@"
