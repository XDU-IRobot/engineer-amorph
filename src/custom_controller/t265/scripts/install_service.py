#!/usr/bin/env python3

import subprocess
import argparse
import sys
import os

SCRIPT_DIR = os.path.dirname(
    os.path.abspath(__file__))

REPOWER_USB_SCRIPT = os.path.join(SCRIPT_DIR, "repower_usb.sh")
AMORPH_CC_SCRIPT = os.path.join(SCRIPT_DIR, "../t265_custom_ctrl/main.py")

REPOWER_USB_SERVICE = f"""[Unit]
After=multi-user.target

[Service]
Type=oneshot
ExecStart=/bin/bash {REPOWER_USB_SCRIPT}
RemainAfterExit=true

[Install]
WantedBy=multi-user.target
"""

AMORPH_CC_SERVICE = f"""[Unit]
After=repower_usb.service
Requires=repower_usb.service

[Service]
Type=simple
ExecStart=/usr/bin/python3 {AMORPH_CC_SCRIPT}
Environment="PYTHONPATH=/home/pi/librealsense/build/wrappers/python:$PYTHONPATH"
Restart=always
User=pi

[Install]
WantedBy=multi-user.target
"""


def install_service(service_name, service_content, start=False):
    service_path = os.path.join(
        "/usr/lib/systemd/system", f"{service_name}.service")
    with open(service_path, "w") as f:
        f.write(service_content)
    subprocess.run(["systemctl", "daemon-reload"])
    subprocess.run(["systemctl", "enable", service_name])
    if start:
        subprocess.run(["systemctl", "start", service_name])
        print(f"Service {service_name} installed and started.")
    else:
        print(f"Service {service_name} installed.")


def uninstall_service(service_name):
    service_path = os.path.join(
        "/usr/lib/systemd/system", f"{service_name}.service")
    if os.path.exists(service_path):
        subprocess.run(["systemctl", "stop", service_name])
        subprocess.run(["systemctl", "disable", service_name])
        os.remove(service_path)
        subprocess.run(["systemctl", "daemon-reload"])
        print(f"Service {service_name} uninstalled.")
    else:
        print(f"Service {service_name} not found.")


def main():
    # check if we have root privileges
    if os.geteuid() != 0:
        print("execute me as root!")
        sys.exit(1)

    parser = argparse.ArgumentParser()
    group = parser.add_mutually_exclusive_group()
    group.add_argument('-i', '--install', action='store_true',
                       help='Install both services')
    group.add_argument('-u', '--uninstall', action='store_true',
                       help='Uninstall the services')
    args = parser.parse_args()

    if args.install:
        if not os.path.exists(REPOWER_USB_SCRIPT):
            print(f"Error: {REPOWER_USB_SCRIPT} does not exist.")
            sys.exit(1)
        if not os.path.exists(AMORPH_CC_SCRIPT):
            print(f"Error: {AMORPH_CC_SCRIPT} does not exist.")
            sys.exit(1)
        install_service("repower_usb", REPOWER_USB_SERVICE)
        install_service("amorph_cc", AMORPH_CC_SERVICE)
    elif args.uninstall:
        uninstall_service("repower_usb")
        uninstall_service("amorph_cc")
    else:
        parser.print_help()


if __name__ == '__main__':
    main()
