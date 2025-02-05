#!/bin/bash
source ~/.bashrc

workspace=$(pwd)


gnome-terminal -t "launcher" -x bash -c "source ~/.bashrc;\
cd ${workspace};\
source devel/setup.bash;roslaunch arm_control X7.launch;exec bash;"




