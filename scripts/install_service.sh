#!/usr/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PASSWORD=$1

echo $PASSWORD | sudo -S cp $SCRIPT_DIR/fluorite_control.service.tmpl /usr/lib/systemd/system/fluorite_control.service
sed -i "s|{PROJECT_PATH}|$SCRIPT_DIR/..|g" /usr/lib/systemd/system/fluorite_control.service
echo $PASSWORD | sudo -S systemctl daemon-reload
echo $PASSWORD | sudo -S systemctl enable fluorite_control.service