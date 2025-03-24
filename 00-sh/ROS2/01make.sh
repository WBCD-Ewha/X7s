#!/bin/bash
source ~/.bashrc
workspace=$(pwd)


#body
gnome-terminal -t "body" -x  bash -c "cd ${workspace}; cd ../..; cd body/ROS2; rm -rf build install log .catkin_workspace ;colcon build; exec bash;"
sleep 0.5
#x7s
gnome-terminal -t "L" -x  bash -c "cd ${workspace}; cd ../..; cd x7s/ROS2/x7s_ws; rm -rf build install log .catkin_workspace src/CMakeLists.txt;colcon build; exec bash;"
sleep 0.5

#VR
gnome-terminal -t "vr" -x  bash -c "cd ${workspace}; cd ../..; cd ARX_VR_SDK/ROS2; rm -rf build install log .catkin_workspace;./port.sh; exec bash;"