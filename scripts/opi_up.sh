#!/bin/bash

if [ "$EUID" -ne 0 ] then 
    echo "execute me as root!"
    exit
fi

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# vaccum pump shut the fuck up
gpio mode 2 pwm
gpio mode 16 pwm
gpio pwm 2 0
gpio pwm 16 0

source ${SCRIPT_DIR}/../install/setup.bash
ros2 launch fluorite_bringup moveit.launch.py
chrt -f 99 ros2 launch fluorite_bringup control.launch.py
