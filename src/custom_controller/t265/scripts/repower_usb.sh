#!/usr/bin/bash

# T265固件存在bug，开机之后需要插拔一次才可以正常使用。这个脚本通过uhubctl重新给USB端口上电来模拟插拔

uhubctl \
  --location 2 \
  --action 2 \
  --repeat 2 \
  --delay 1