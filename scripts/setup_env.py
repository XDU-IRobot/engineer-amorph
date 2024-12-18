#!/usr/bin/env python3

import subprocess
import argparse
import os
import sys

from sdbus_block.networkmanager import (
    NetworkManager,
    NetworkDeviceGeneric,
    IPv4Config,
)
from sdbus import sd_bus_open_system
system_bus = sd_bus_open_system()
nm = NetworkManager(system_bus)

# todo
def set_network_static_ip(interface: str, ip: str, netmask: int) -> None:
    pass


def setup_jetson():
    set_network_static_ip("eth0", "10.0.3.101", 24)
    print("network ok: eth0 set to 10.0.3.101/24")


def setup_opi():
    CAN_MODULES = ["can", "mttcan", "can_raw"]
    NETWORK_RULE = r"""
[Match]
Name=can*

[CAN]
BitRate=1M
    """
    SERIAL_RULES = {
    "rm-remote": 'KERNEL=="ttyS0", MODE="0666", GROUP:="dialout", SYMLINK+="sbus_remote"',
    "rm-fpv": 'KERNEL=="ttyS1", MODE="0666", GROUP:="dialout", SYMLINK+="rm_fpv"',
    "hwt606": 'KERNEL=="ttyS4", MODE="0666", GROUP:="dialout", SYMLINK+="hwt606"',}
    
    DTOVERLAYS = {"can1-m1", "can2-m1", "uart0-m2", "uart1-m1", "uart4-m0", "pwm1-m2", "pwm15-m2"}
    
    ## LOAD KERNEL MODULES
    with open("/etc/modules-load.d/can.conf", "w") as f:
        f.write("\n".join(CAN_MODULES))
    subprocess.run(["systemctl", "restart", "systemd-modules-load"])
    print("kernel modules ok")
    
    ## SYSTEMD NETWORKD RULES FOR CAN
    with open("/etc/systemd/network/100-can.network", "w") as f:
        f.write(NETWORK_RULE)
    subprocess.run(["systemctl", "restart", "systemd-networkd"])
    subprocess.run(["systemctl", "enable", "systemd-networkd"])
    print("systemd-networkd rules ok")
    
    ## UDEV RULES FOR SERIAL PORTS
    for k, v in SERIAL_RULES.items():
        with open(f"/etc/udev/rules.d/100-{k}.rules", "w") as f:
            f.write(v)
    subprocess.run(["udevadm", "control", "--reload-rules"])
    subprocess.run(["udevadm", "trigger"])
    print("udev serial rules ok")
    
    ## NETWORK SETTINGS
    set_network_static_ip("eth0", "10.0.3.102", 24)
    print("network ok: eth0 set to 10.0.3.102/24")
    
    ## DTOVERLAYS
    if not os.path.exists("/boot/orangepiEnv.txt"):
        print("no /boot/orangepiEnv.txt file found, not doing any change to device tree overlays")
        sys.exit(1)
    with open("/boot/orangepiEnv.txt", "r") as f:
        lines = f.readlines()
        overlays = None
        for line in lines:
            if line.startswith("overlays="):
                overlays = set(line.strip().split("=")[1].split(","))
                break
        if overlays is None:
            print("no overlays= line found in /boot/orangepiEnv.txt file, exit")
            sys.exit(1)
        new_overlays_str = f"overlays={' '.join(DTOVERLAYS.union(overlays))}"
        for i, line in enumerate(lines):
            if line.startswith("overlays="):
                lines[i] = new_overlays_str
                break
    with open("/boot/orangepiEnv.txt", "w") as f:
        f.writelines(lines)
    print("device tree overlays ok")


def main():
    # do we have root permission?
    if os.geteuid() != 0:
        print("execute me as root!")
        sys.exit(1)
        
    try:
        input("""
            CONFIRM AGAIN that you actually want to run this script, 
            this is going to tweak some important system configurations!!!
            
            Enter = continue, Ctrl+C = exit:
            """)
    except KeyboardInterrupt:
        sys.exit(0)
    
    parser = argparse.ArgumentParser()
    # -j --jetson/-o --orangepi
    # if none of them is provided or both are provided, print help message
    group = parser.add_mutually_exclusive_group()
    group.add_argument("-j", "--jetson", action="store_true", help="Setup for Jetson")
    group.add_argument("-o", "--orangepi", action="store_true", help="Setup for Orange Pi")
    args = parser.parse_args()
    if args.jetson:
        setup_jetson()
    elif args.orangepi:
        setup_opi()
    else:
        parser.print_help()

if __name__ == "__main__":
    main()