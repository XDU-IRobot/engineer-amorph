#!/bin/bash
source /opt/ros/rolling/setup.bash
colcon build \
  --symlink-install \
  --packages-select amorph_bringup amorph_vision amorph_onboard_transport orbbec_camera orbbec_camera_msgs orbbec_description \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-Wno-error=stringop-overflow -Wno-error"