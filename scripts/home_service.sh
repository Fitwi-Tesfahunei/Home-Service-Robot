#!/bin/sh
xterm -e "source /home/workspace/catkin_ws/devel/setup.bash" &
sleep 5
xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 5
xterm -e "roslaunch turtlebot_gazebo amcl_demo.launch" &
sleep 5
xterm -e "roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 10
xterm -e "roslaunch add_virtual_objects add_virtual_objects.launch" &
sleep 15
xterm -e "roslaunch pick_objects pick_objects.launch"
