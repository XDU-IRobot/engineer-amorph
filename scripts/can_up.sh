#!/bin/bash

CAN=$1

echo "Bringing up CAN interface: $CAN"

modprobe can
modprobe mttcan
modprobe can_raw

ip link set can0 down
ip link set can0 type can \
    bitrate 1000000 \
    dbitrate 4800000 \
    fd on
ip link set can0 up
