#!/usr/bin/env python3

import argparse
import subprocess

COMMON_ARGS = ["--merge-install", "--symlink-install", "--cmake-args",
               "-DCMAKE_BUILD_TYPE=RelWithDebInfo", "-DCMAKE_EXPORT_COMPILE_COMMANDS=Yes"]


def main():
    parser = argparse.ArgumentParser(description='Build script')
    # -j --jetson/-o --orangepi/-d --debug
    # if none of them is provided or both are provided, print help message
    group = parser.add_mutually_exclusive_group()
    group.add_argument('-j', '--jetson', action='store_true',
                       help='Build for Jetson')
    group.add_argument('-o', '--orangepi', action='store_true',
                       help='Build for Orange Pi')
    group.add_argument('-d', '--debug', action='store_true',
                       help='Build all packages(for debug/development)')
    args = parser.parse_args()
    if args.jetson:
        subprocess.run(["colcon", "build", "--paths", "src/common/*",
                       "src/jetson/*", "--packages-ignore", "keyboard", *COMMON_ARGS])
    elif args.orangepi:
        subprocess.run(["colcon", "build", "--paths",
                       "src/common/*", "src/opi/*", *COMMON_ARGS])
    elif args.debug:
        subprocess.run(["colcon", "build", *COMMON_ARGS])
    else:
        parser.print_help()


if __name__ == '__main__':
    main()
