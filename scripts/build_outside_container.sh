#!/bin/bash
source /opt/ros/foxy/setup.bash
source .venv/bin/activate
colcon build --symlink-install --packages-select energy_unit_locator