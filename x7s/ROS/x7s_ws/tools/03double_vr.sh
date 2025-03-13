#!/bin/bash
source ~/.bashrc
workspace=$(pwd)

# CAN
gnome-terminal -t "can1" -x bash -c "cd ${workspace}/ARX_CAN/arx_can; ./arx_can1.sh; exec bash;"
# R5Pro
gnome-terminal -t "R5Pro" -x  bash -c "cd ${workspace};cd ROS/R5Pro_ws; source devel/setup.bash && roslaunch arx_r5pro_controller open_vr_double_arm.launch; exec bash;"

# VR
gnome-terminal -t "unity_tcp" -x bash -c "cd ${workspace};cd ..;cd ARX_VR_SDK/ROS ;source devel/setup.bash && rosrun serial_port serial_port_node;exec bash;"
sleep 1
gnome-terminal -t "arx5_pos_cmd" -x bash -c "cd ${workspace};cd ..;cd ARX_VR_SDK/ROS ;source devel/setup.bash && rostopic echo /ARX_VR_L;exec bash;"
