#!/bin/sh

# Deploy Turtlebot in personal environment
xterm -e "export ROBOT_INITIAL_POSE='-x 0.0 -y 0.0 -Y -1.57079633' && roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$(rospack find home_service_robot)/world/AndrewOffice2.world" &
sleep 8

# amcl_demo to loclize the robot
xterm -e "roslaunch turtlebot_navigation amcl_demo.launch map_file:=$(rospack find home_service_robot)/map/AndrewOffice2.yaml" &
sleep 5

# view_navigation to observe the map in rviz
xterm -e "roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5

# Deploy add_markers node
xterm -e "rosrun add_markers add_markers_node"
