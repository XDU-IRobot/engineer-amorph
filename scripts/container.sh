#!/bin/bash

command=$1

if [ "$command" == "create" ]; then
    docker run --privileged --net=host --name amorph_container -v /home/nvidia:/home/nvidia -it ros:jazzy
elif [ "$command" == "start" ]; then
    docker start amorph_container
elif [ "$command" == "attach" ]; then
    docker attach --detach-keys="ctrl-x" amorph_container
else
    echo "Usage: $0 {create|start|attach}"
    exit 1
fi
