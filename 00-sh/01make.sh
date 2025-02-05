#!/bin/bash
workspace=$(pwd)

source ~/.bashrc



gnome-terminal -t "L" -x  bash -c "cd ${workspace}; cd ..; cd ARX_X7s_SDK/ARX7_L; rm -rf build devel .catkin_workspace src/CMakeLists.txt; catkin_make clean ;catkin_make; exec bash;"
sleep 0.5
gnome-terminal -t "R" -x  bash -c "cd ${workspace}; cd ..; cd ARX_X7s_SDK/ARX7_R; rm -rf build devel .catkin_workspace src/CMakeLists.txt; catkin_make clean ;catkin_make; exec bash;"
sleep 0.5
# gnome-terminal -t "ARX7_L" -x sudo bash -c "cd ${workspace}/ARX7_L; source devel/setup.bash && roslaunch arm_control X7.launch; exec bash;"
# sleep 1
# gnome-terminal -t "ARX7_R" -x sudo bash -c "cd ${workspace}/ARX7_R; source devel/setup.bash && roslaunch arm_control X7.launch; exec bash;"


gnome-terminal -t "body" -x  bash -c "cd ${workspace}; cd ..; cd X7s_Body_SDK; rm -rf build devel .catkin_workspace; catkin_make clean ;catkin_make; exec bash;"
sleep 0.5
gnome-terminal -t "vr" -x  bash -c "cd ${workspace}; cd ..; cd vrserialportsdk; rm -rf build devel .catkin_workspace;./port.sh; exec bash;"

