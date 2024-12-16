#!/usr/bin/env python3

import subprocess
import argparse
import os
import sys

def setup_jetson():
    # "Wired connection 1" exists probably, remove it first
    subprocess.run(['nmcli', 'con', 'delete', 'Wired connection 1'])
    subprocess.run(['nmcli', 'con', 'add', 'type', 'ethernet', 'ifname', 'eth0', 'con-name', r'Wired connection 1'])
    # static ip, 10.0.3.102/24
    subprocess.run(['nmcli', 'con', 'modify', 'Wired connection 1', 'ipv4.method', 'manual', 'ipv4.addresses', '10.0.3.102/24'])
    subprocess.run(['nmcli', 'con', 'up', 'Wired connection 1'])
    print('network ok: eth0 set to 10.0.3.102/24')

def setup_opi():
    CAN_MODULES = r'''
can
can_raw
mttcan
    '''
    NETWORK_RULE = r'''
[Match]
Name=can*

[CAN]
BitRate=1M
    '''
    SERIAL_RULE = r'''
KERNEL=="ttyS3", MODE="0666", GROUP:="dialout", SYMLINK+="rm_remote"
    '''
    
    with open('/etc/modules-load.d/can.conf', 'w') as f:
        f.write(CAN_MODULES)
    subprocess.run(['systemctl', 'restart', 'systemd-modules-load'])
    print('can modules requirements ok')
    
    with open('/etc/systemd/network/99-can.network', 'w') as f:
        f.write(NETWORK_RULE)
    subprocess.run(['systemctl', 'restart', 'systemd-networkd'])
    subprocess.run(['systemctl', 'enable', 'systemd-networkd'])
    print('network rules ok')
    
    with open('/etc/udev/rules.d/99-fluorite-serial.rules', 'w') as f:
        f.write(SERIAL_RULE)
    subprocess.run(['udevadm', 'control', '--reload-rules'])
    subprocess.run(['udevadm', 'trigger'])
    print('serial rules ok')
    
    # "Wired connection 1" exists probably, remove it first
    subprocess.run(['nmcli', 'con', 'delete', 'Wired connection 1'])
    subprocess.run(['nmcli', 'con', 'add', 'type', 'ethernet', 'ifname', 'eth0', 'con-name', r'Wired connection 1'])
    # static ip, 10.0.3.101/24
    subprocess.run(['nmcli', 'con', 'modify', 'Wired connection 1', 'ipv4.method', 'manual', 'ipv4.addresses', '10.0.3.101/24'])
    subprocess.run(['nmcli', 'con', 'up', 'Wired connection 1'])
    print('network ok: eth0 set to 10.0.3.101/24')


def main():
    # do we have root permission?
    if os.geteuid() != 0:
        print('execute me as root!')
        sys.exit(1)
    
    parser = argparse.ArgumentParser()
    # -j --jetson/-o --orangepi
    # if none of them is provided or both are provided, print help message
    group = parser.add_mutually_exclusive_group()
    group.add_argument('-j', '--jetson', action='store_true', help='Setup for Jetson')
    group.add_argument('-o', '--orangepi', action='store_true', help='Setup for Orange Pi')
    args = parser.parse_args()
    if args.jetson:
        setup_jetson()
    elif args.orangepi:
        setup_opi()
    else:
        parser.print_help()

if __name__ == '__main__':
    main()