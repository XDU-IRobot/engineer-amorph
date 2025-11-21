#!/bin/bash
source /opt/ros/rolling/setup.bash
source .venv/bin/activate
colcon build --symlink-install --packages-select energy_unit_locator amorph_msgs --install-base install_native