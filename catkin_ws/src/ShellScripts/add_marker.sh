#!/bin/sh

xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/cl/AAAProjects/AAAUdacity/roboND2/Proj5_HomeServiceRobot/P5_Root/catkin_ws/src/World/MyWorld.world" &
sleep 5

ws=/home/cl/AAAProjects/AAAUdacity/roboND2/Proj5_HomeServiceRobot/P5_Root/catkin_ws
world_file=${ws}/src/World/MyWorld.world
map_file=${ws}/src/World/MyWorldMap.yaml
xterm -e "roslaunch turtlebot_gazebo amcl_demo.launch map_file:=${map_file} _3d_sensor:=kinect" &
#xterm -e "roslaunch turtlebot_gazebo amcl_demo.launch " &
sleep 5

xterm -e "roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5

#xterm  -e  "rosrun add_markers add_markers" &
#rosrun add_markers add_markers