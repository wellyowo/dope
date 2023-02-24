#! /bin/bash

if [ "$1" ]; then
    echo "ROS MASRER $1"
    export ROS_MASTER_URI=http://$1:11311
else
    echo "ROS MASRER 10.42.0.2"
    export ROS_MASTER_URI=http://10.42.0.2:11311
fi

if [ "$2" ]; then
    echo "ROS IP $2"
    export ROS_IP=$2
else
    echo "ROS IP 10.42.0.2"
    export ROS_IP=10.42.0.2
fi
