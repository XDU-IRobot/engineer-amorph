#!/usr/bin/env python3

import argparse
import subprocess

COMMON_ARGS = ["--merge-install", "--symlink-install", "--cmake-args", "-DCMAKE_BUILD_TYPE=RelWithDebInfo", "-DCMAKE_EXPORT_COMPILE_COMMANDS=Yes"]

def main():
    parser = argparse.ArgumentParser(description='Build script')
    # -j --jetson/-o --orangepi
    # if none of them is provided or both are provided, print help message
    group = parser.add_mutually_exclusive_group()
    group.add_argument('-j', '--jetson', action='store_true', help='Build for Jetson')
    group.add_argument('-o', '--orangepi', action='store_true', help='Build for Orange Pi')
    args = parser.parse_args()
    if args.jetson:
        subprocess.run(["colcon", "build", "--paths", "src/common/*", "src/jetson/*", *COMMON_ARGS])
    elif args.orangepi:
        subprocess.run(["colcon", "build", "--paths", "src/common/*", "src/opi/*", *COMMON_ARGS])
    else:
        parser.print_help()

if __name__ == '__main__':
    main()