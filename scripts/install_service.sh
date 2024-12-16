#!/usr/bin/bash

if [ "$EUID" -ne 0 ] then 
    echo "execute me as root!"
    exit
fi

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

cp $SCRIPT_DIR/fluorite_opi.service.tmpl /usr/lib/systemd/system/fluorite_opi.service
cp $SCRIPT_DIR/fluorite_jetson.service.tmpl /usr/lib/systemd/system/fluorite_jetson.service
sed -i "s|{PROJECT_PATH}|$SCRIPT_DIR/..|g" /usr/lib/systemd/system/fluorite_opi.service
sed -i "s|{PROJECT_PATH}|$SCRIPT_DIR/..|g" /usr/lib/systemd/system/fluorite_jetson.service
