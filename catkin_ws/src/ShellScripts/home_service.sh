#!/bin/bash
# Only works with bash shell!

#################### Paths & Files ############################
# Find the path to the catkin_ws folder.
# This is based on the assumption that this script is located 2 folders down from catkin_ws,
# For example:
# $CATKIN_WS_DIR/src/ShellScripts/this_script.sh
THIS_SCRIPT_DIR="$( cd "$(dirname "$0")" ; pwd -P )"
CATKIN_WS_DIR="$( cd "$( dirname "${THIS_SCRIPT_DIR}" )/.." >/dev/null && pwd )"
echo "CATKIN_WS_DIR == $CATKIN_WS_DIR"
# My mint laptop WS
# CATKIN_WS_DIR=/home/cl/AAAProjects/AAAUdacity/roboND2/Proj5_HomeServiceRobot/P5_Root/catkin_ws

# This is probably not needed, as reviewer should have sourced properly already... But shouldn't hurt.
source $CATKIN_WS_DIR/devel/setup.bash

WORLD_FILE=${CATKIN_WS_DIR}/src/World/MyWorld.world
MAP_FILE=${CATKIN_WS_DIR}/src/World/MyWorldMap.yaml
RVIZCFG_FILE=${CATKIN_WS_DIR}/src/RVizConfig/home_service.rviz

#################### Launchers ############################
xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=${WORLD_FILE}" &
sleep 3

xterm -e "roslaunch turtlebot_gazebo amcl_demo.launch map_file:=${MAP_FILE} " &
sleep 3

xterm -e "rosrun rviz rviz -d ${RVIZCFG_FILE}" &
sleep 5

xterm -e "rosrun add_markers add_markers" &


