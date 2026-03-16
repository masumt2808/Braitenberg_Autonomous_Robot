#!/bin/bash

# allow docker to access your display
xhost +local:docker

docker compose -f "$(dirname "$0")/docker-compose.yml" up --build -d

echo ""
echo "Container is running. To open a terminal inside it:"
echo "  docker exec -it braitenberg_ws-braitenberg-1 bash"
echo ""
echo "Then run the simulation:"
echo "  ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py"
echo "  ros2 run autonomous_robot obstacle_avoidance"
