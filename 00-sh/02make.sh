#!/bin/bash
workspace=$(pwd)

source ~/.bashrc

gnome-terminal -t "R5" -x  bash -c "cd ${workspace}; cd ..; cd vrserialportsdk; rm -rf build devel .catkin_workspace;catkin_make; exec bash;"

