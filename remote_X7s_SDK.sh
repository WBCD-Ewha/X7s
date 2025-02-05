#!/bin/bash

# workspace=$(pwd)

workspace=/home/arx/桌面/X7s_ALL_IN_ONE

source ~/.bashrc

# CAN
gnome-terminal -t "can0" -x sudo bash -c "cd ${workspace}/ARX_CAN/arx_can; ./arx_can0.sh; exec bash;"
sleep 0.1
gnome-terminal -t "can1" -x sudo bash -c "cd ${workspace}/ARX_CAN/arx_can; ./arx_can1.sh; exec bash;"
sleep 0.1
gnome-terminal -t "can5" -x sudo bash -c "cd ${workspace}/ARX_CAN/arx_can; ./arx_can5.sh; exec bash;"
sleep 1

# X7
gnome-terminal -t "body" -x sudo bash -c "cd ${workspace}/X7s_Body_SDK; source devel/setup.bash && roslaunch arm_control arx5.launch; exec bash;"
sleep 3
gnome-terminal -t "L" -x sudo bash -c "cd ${workspace}/ARX_X7s_SDK/ARX7_L; source devel/setup.bash && roslaunch arm_control X7.launch; exec bash;"
sleep 1
gnome-terminal -t "R" -x sudo bash -c "cd ${workspace}/ARX_X7s_SDK/ARX7_R; source devel/setup.bash && roslaunch arm_control X7.launch; exec bash;"
sleep 1
# gnome-terminal -t "pub" -x bash -c "source ~/.bashrc;rosrun joy joy_node;exec bash;"
# VR
gnome-terminal -t "unity_tcp" -x bash -c "cd ${workspace}/vrserialportsdk;source devel/setup.bash;rosrun serial_port serial_port;exec bash;"
sleep 1
gnome-terminal -t "arx5_pos_cmd" -x bash -c "cd ${workspace}/vrserialportsdk;source devel/setup.bash;rostopic echo /ARX_VR_L;exec bash;"
