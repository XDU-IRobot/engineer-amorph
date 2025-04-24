#!/usr/bin/env python3

import subprocess
import argparse
import sys
import os


PROJECT_ROOT = os.path.dirname(
    os.path.abspath(os.path.join(__file__, "..")))

OPI_SERVICE = f"""[Unit]
After=systemd-networkd.service
Requires=systemd-networkd.service

[Service]
Type=simple
ExecStartPre={PROJECT_ROOT}/scripts/wait_for_can_up.sh
ExecStart={PROJECT_ROOT}/scripts/opi_up.sh
User=root

[Install]
WantedBy=multi-user.target
"""

OPI_VACCUM_PUMP_STFU_SERVICE = f"""[Unit]
After=multi-user.target

[Service]
Type=oneshot
ExecStart={PROJECT_ROOT}/scripts/vaccum_pump_stfu.sh
User=root

[Install]
WantedBy=multi-user.target
"""

CUSTOM_CTRL_SERVICE = f"""[Unit]
After=multi-user.target

[Service]
Type=simple
ExecStart={PROJECT_ROOT}/custom_ctrl_build/fluorite_custom_controller
User=root
StartLimitIntervalSec=0
Restart=always
RestartSec=1

[Install]
WantedBy=multi-user.target
"""


def install_service(service_name, service_content, start=False):
    service_path = os.path.join(
        r"/usr/lib/systemd/system", f"{service_name}.service")
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
        r"/usr/lib/systemd/system", f"{service_name}.service")
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

    parser = argparse.ArgumentParser(
        description='Service installation script for fluorite robot')
    # -j --jetson/-o --orangepi/-c --customctrl
    # if none of them is provided or more than one is provided, print help message
    group = parser.add_mutually_exclusive_group()
    group.add_argument('-j', '--jetson', action='store_true',
                       help='Install service for Jetson')
    group.add_argument('-o', '--orangepi', action='store_true',
                       help='Install service for Orange Pi')
    group.add_argument('-c', '--customctrl', action='store_true',
                       help='Install service for T265 custom controller(the rpi)')
    group.add_argument('-u', '--uninstall', action='store_true',
                       help='Uninstall services')
    args = parser.parse_args()

    if args.jetson:
        pass  # WIP ;DDDDDD
    elif args.orangepi:
        install_service("fluorite_opi", OPI_SERVICE)
        install_service("fluorite_vaccum_pump_stfu",
                        OPI_VACCUM_PUMP_STFU_SERVICE)
    elif args.customctrl:
        install_service("fluorite_custom_controller",
                        CUSTOM_CTRL_SERVICE, start=True)
    elif args.uninstall:
        uninstall_service("fluorite_opi")
        uninstall_service("fluorite_vaccum_pump_stfu")
        uninstall_service("fluorite_jetson")
        uninstall_service("fluorite_custom_controller")
    else:
        parser.print_help()


if __name__ == '__main__':
    main()
