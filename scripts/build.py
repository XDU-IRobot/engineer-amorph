#!/usr/bin/env python3

import argparse
import subprocess

COMMON_ARGS = ["--merge-install", "--symlink-install", "--cmake-args",
               "-DCMAKE_BUILD_TYPE=RelWithDebInfo", "-DCMAKE_EXPORT_COMPILE_COMMANDS=Yes"]


def main():
    parser = argparse.ArgumentParser(description='Build script')
    # -j --jetson/-o --orangepi/-c --customctrl/-d --development
    # if none of them is provided or both are provided, print help message
    group = parser.add_mutually_exclusive_group()
    group.add_argument('-j', '--jetson', action='store_true',
                       help='Build for Jetson')
    group.add_argument('-o', '--orangepi', action='store_true',
                       help='Build for Orange Pi')
    group.add_argument('-c', '--customctrl', action='store_true',
                       help='Build for custom controller(on the raspberry pi)')
    group.add_argument('-d', '--development', action='store_true',
                       help='Build all packages except for custom controller(for debug/development/ci)')
    args = parser.parse_args()

    if args.jetson:
        status = subprocess.run(["colcon", "build", "--base-paths", "src/common/**",
                       "src/jetson/**", *COMMON_ARGS])
        if status.returncode != 0:
            exit(1)
    elif args.orangepi:
        status = subprocess.run(["colcon", "build", "--base-paths",
                       "src/common/**", "src/opi/**", "--packages-ignore", "keyboard", *COMMON_ARGS])
        if status.returncode != 0:
            exit(1)
    elif args.customctrl:
        status = subprocess.run(["cmake", "-B", "custom_ctrl_build", "-S",
                       "src/custom_controller/t265", "-DCMAKE_BUILD_TYPE=Release"])
        if status.returncode != 0:
            exit(1)
        status = subprocess.run(["cmake", "--build", "custom_ctrl_build"])
        if status.returncode != 0:
            exit(1)
    elif args.development:
        status = subprocess.run(["colcon", "build", "--base-paths",
                       "src/**", *COMMON_ARGS])
        if status.returncode != 0:
            exit(1)
    else:
        parser.print_help()


if __name__ == '__main__':
    main()
