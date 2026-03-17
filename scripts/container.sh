#!/bin/bash

command=$1

if [ "$command" == "create" ]; then
    docker run --privileged --net=host --name amorph_container \
        -v /home/nvidia:/home/nvidia \
        --restart unless-stopped \
        amorph_jazzy_saved \
        /bin/bash -c "source /home/nvidia/engineer-amorph/install/setup.bash && ros2 launch amorph_bringup in_container.launch.py"
elif [ "$command" == "start" ]; then
    docker start amorph_container
elif [ "$command" == "attach" ]; then
    docker attach --detach-keys="ctrl-x" amorph_container
else
    echo "Usage: $0 {create|start|attach}"
    exit 1
fi
