#!/bin/bash

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA

export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/MyCode/Project/Drone/ROS2/config/FastDDS/config.xml
export FASTDDS_DEFAULT_PROFILES_FILE=$HOME/MyCode/Project/Drone/ROS2/config/FastDDS/config.xml
export RMW_FASTRTPS_USE_QOS_FROM_XML=1