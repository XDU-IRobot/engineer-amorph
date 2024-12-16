#!/usr/bin/bash

if [ "$EUID" -ne 0 ] then 
    echo "execute me as root!"
    exit
fi

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

systemctl stop fluorite_opi.service
systemctl stop fluorite_jetson.service
systemctl disable fluorite_opi.service
systemctl disable fluorite_jetson.service
rm /usr/lib/systemd/system/fluorite_opi.service
rm /usr/lib/systemd/system/fluorite_jetson.service
systemctl daemon-reload