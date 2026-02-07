#!/bin/bash

CAN=$1

echo "Bringing up CAN interface: $CAN"

modprobe can
modprobe mttcan
modprobe can_raw
modprobe peak_usb

ip link set can0 down
ip link set can0 type can \
    bitrate 1000000 \
    dbitrate 3200000 \
    fd on
ip link set can0 txqueuelen 1000
ip link set can0 up
