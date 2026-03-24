#!/bin/zsh

ALL_GOOD=1
GLOBAL_ENV="$HOME/MyCode/Project/Drone/ROS2/scripts/global.sh"
ROS_SETUP="/opt/ros/jazzy/setup.zsh"
ROS_LOCAL_SETUP="$HOME/MyCode/Project/Drone/ROS2/install/local_setup.zsh"
ROS_DDS_CONFIG="$HOME/MyCode/Project/Drone/ROS2/config/FastDDS/fast_dds_setup.sh"
ROS_PYVENV="$HOME/MyCode/Project/Drone/ROS2/pyvenv/bin/activate"

if [ -f "$GLOBAL_ENV" ]; then
    source "$GLOBAL_ENV"
else
    echo '[Missing Global ENV setup]'
    ALL_GOOD=0
fi

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

if [ -f "$ROS_DDS_CONFIG" ]; then
    source "$ROS_DDS_CONFIG"
else
    echo '[FastDDS Setup missing]'
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
    # echo $'\033[32m <| Setup complete! |>\033[0m'
    # clear
    source $HOME/.config/terminal-tag/ROS2.sh
else
    echo $'\033[31m <| Setup incomplete! |>\033[0m'
    source $HOME/.config/terminal-tag/ROS2-incomplete.sh
fi