#!/usr/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PASSWORD=$1

echo $PASSWORD | sudo -S systemctl stop fluorite_control.service
echo $PASSWORD | sudo -S systemctl disable fluorite_control.service
echo $PASSWORD | sudo -S rm /usr/lib/systemd/system/fluorite_control.service 
echo $PASSWORD | sudo -S systemctl daemon-reload