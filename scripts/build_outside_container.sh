#!/bin/bash
source /opt/ros/rolling/setup.bash
colcon build --symlink-install --packages-select energy_unit_locator --install-base install_native --build-base build_native