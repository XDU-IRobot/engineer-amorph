#!/bin/bash

# if [ "$EUID" -ne 0 ] then 
#     echo "execute me as root!"
#     exit
# fi

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

source ${SCRIPT_DIR}/../install/setup.bash
ros2 launch fluorite_bringup opi.py
# chrt -f 99 ros2 launch fluorite_bringup control.launch.py
