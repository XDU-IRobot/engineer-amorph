#!/usr/bin/env python3

import sys
import subprocess

CMAKE_ARGS = [
    "-DCMAKE_BUILD_TYPE=RelWithDebInfo",
    "-DCMAKE_EXPORT_COMPILE_COMMANDS=Yes",
    "-DBUILD_TESTS=No",
    "-DBUILD_TESTING=No",
]


def main():
    colcon_cmd = [sys.executable, "-m", "colcon"]
    args = [
        "build",
        "--symlink-install",
        "--cmake-args",
        *CMAKE_ARGS,
    ]
    result = subprocess.run(colcon_cmd + args)
    sys.exit(result.returncode)


if __name__ == "__main__":
    main()
