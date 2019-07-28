#!/bin/bash
#conda init bash && conda activate /opt/robond/py2/
#sudo apt update && sudo apt upgrade -y
source /home/workspace/catkin_ws/devel/setup.bash

# Launch world wit turtlebot:
xterm -e " export GAZEBO_MODEL_PATH=/home/workspace/catkin_ws/src/my_robot/models; roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/workspace/catkin_ws/src/my_robot/worlds/my_world.world " &
sleep 5

# Launch gmapping:
xterm -e " roslaunch turtlebot_gazebo gmapping_demo.launch " &
sleep 5

# Launch rviz:
xterm -e " roslaunch turtlebot_rviz_launchers view_navigation.launch " &
sleep 5

# Launch teleop:
xterm -e " roslaunch turtlebot_teleop keyboard_teleop.launch " &
