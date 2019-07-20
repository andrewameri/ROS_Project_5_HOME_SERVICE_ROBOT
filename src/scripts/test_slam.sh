#!/bin/sh

# Deploy Turtlebot in personal environment
xterm -e "export ROBOT_INITIAL_POSE='-x 0.0 -y 0.0 -Y -1.57079633' && roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$(rospack find home_service_robot)/world/AndrewOffice2.world" &
sleep 10

# Run slam_gmapping to perfrom SLAM
xterm -e "export TURTLEBOT_3D_SENSOR='kinect' && roslaunch turtlebot_gazebo gmapping_demo.launch" &
sleep 5

# view_navigation to observe the map in rviz
xterm -e "roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5

# keyboard_teleop to manually control the robot with keyboard commands
xterm -e "roslaunch turtlebot_teleop keyboard_teleop.launch"
