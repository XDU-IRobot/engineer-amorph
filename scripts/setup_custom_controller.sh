#!/usr/bin/bash

if [ "$EUID" -ne 0 ]; then
    echo "execute me as root!"
    exit
fi

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

cp $SCRIPT_DIR/fluorite-custom-controller.service.tmpl /usr/lib/systemd/system/fluorite-custom-controller.service
sed -i "s|{PROJECT_PATH}|$SCRIPT_DIR/..|g" /usr/lib/systemd/system/fluorite-custom-controller.service

systemctl daemon-reload
systemctl enable fluorite-custom-controller.service
systemctl start fluorite-custom-controller.service