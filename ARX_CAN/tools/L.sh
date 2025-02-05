#!/bin/bash
source ~/.bashrc

cansend can0 001#FFFFFFFFFFFFFFFC
sleep 0.1
cansend can0 002#FFFFFFFFFFFFFFFC
sleep 0.1
cansend can0 003#FFFFFFFFFFFFFFFC
sleep 0.1
cansend can0 004#FFFFFFFFFFFFFFFC
sleep 0.1
cansend can0 005#FFFFFFFFFFFFFFFC
sleep 0.1
cansend can0 006#FFFFFFFFFFFFFFFC
sleep 0.1
cansend can0 007#FFFFFFFFFFFFFFFC
sleep 0.1
cansend can0 008#FFFFFFFFFFFFFFFC
sleep 0.1
echo "init done"


cansend can0 001#FFFFFFFFFFFFFFFE
sleep 0.1
cansend can0 002#FFFFFFFFFFFFFFFE
sleep 0.1
cansend can0 003#FFFFFFFFFFFFFFFE
sleep 0.1
cansend can0 004#FFFFFFFFFFFFFFFE
sleep 0.1
cansend can0 005#FFFFFFFFFFFFFFFE
sleep 0.1
cansend can0 006#FFFFFFFFFFFFFFFE
sleep 0.1
cansend can0 007#FFFFFFFFFFFFFFFE
sleep 0.1
cansend can0 008#FFFFFFFFFFFFFFFE
sleep 0.1
echo "cali done"
