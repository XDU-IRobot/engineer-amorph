#!/bin/bash
source /opt/ros/rolling/setup.bash
colcon build --symlink-install --packages-select amorph_vision --install-base install_native --build-base build_native