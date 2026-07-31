#!/usr/bin/env python3

import argparse
import os
import sys
import subprocess

CMAKE_ARGS = [
    "-DCMAKE_BUILD_TYPE=RelWithDebInfo",
    "-DCMAKE_EXPORT_COMPILE_COMMANDS=Yes",
    "-DBUILD_TESTS=No",
    "-DBUILD_TESTING=No",
]


def main():
    parser = argparse.ArgumentParser(description="Build the workspace with colcon")
    parser.add_argument(
        "-j", "--jobs",
        type=int,
        default=None,
        help="Number of parallel build jobs (sets MAKEFLAGS=-j<N>)",
    )
    parsed = parser.parse_args()

    env = os.environ.copy()
    if parsed.jobs is not None:
        env["MAKEFLAGS"] = f"-j{parsed.jobs}"

    colcon_cmd = [sys.executable, "-m", "colcon"]
    args = [
        "build",
        "--symlink-install",
        "--cmake-args",
        *CMAKE_ARGS,
    ]
    result = subprocess.run(colcon_cmd + args, env=env)
    sys.exit(result.returncode)


if __name__ == "__main__":
    main()
