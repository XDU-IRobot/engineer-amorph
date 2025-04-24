#!/bin/bash

# 检查 can0 和 can1 是否均为 UP

check_can() {
    ip link show can0 | grep -q "state UP" && \
    ip link show can1 | grep -q "state UP"
}

timeout=30
while ! check_can; do
    if [ "$timeout" -le 0 ]; then
        echo "Timeout: CAN interfaces not UP!"
        exit 1
    fi
    sleep 1
    ((timeout--))
done

exit 0