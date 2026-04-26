#!/bin/zsh

ALL_GOOD=1
ROS_SETUP="/opt/ros/jazzy/setup.zsh"
ROS_LOCAL_SETUP="$HOME/MyCode/Project/Drone/ROS2/install/local_setup.zsh"
ROS_PYVENV="$HOME/MyCode/Project/Drone/ROS2/pyvenv/bin/activate"

if [ -f "$ROS_SETUP" ]; then
    source "$ROS_SETUP"
else
    echo '[ROS2 Jazzy not found]'
    ALL_GOOD=0
fi

if [ -f "$ROS_LOCAL_SETUP" ]; then
    source "$ROS_LOCAL_SETUP"
else
    echo '[Local setup missing]'
    ALL_GOOD=0
fi

if [ -f "$ROS_PYVENV" ]; then
    source "$ROS_PYVENV"
    export PYTHONNOUSERSITE=1
else
    echo '[Python pyvenv missing]'
    ALL_GOOD=0
fi

cd $HOME/MyCode/Project/Drone/ROS2

if [ "$ALL_GOOD" -eq 1 ]; then
    echo $'Init >>> \033[32mSourcing complete! \033[0m'
else
    echo $'Init >>> \033[31mSourcing incomplete! \033[0m'
fi