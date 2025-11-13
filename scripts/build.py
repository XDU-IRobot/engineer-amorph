#!/usr/bin/env python3

import os
import sys
import subprocess

IGNORE_PACKAGES = [
    "moveit_task_constructor_demo",
]
MTC_PACKAGES = [
    "rviz_marker_tools",
    "moveit_task_constructor_core",
    "moveit_task_constructor_msgs",
    "moveit_task_constructor_capabilities",
    "moveit_task_constructor_visualization",
]

CMAKE_ARGS = [
    "-DCMAKE_BUILD_TYPE=RelWithDebInfo",
    "-DCMAKE_EXPORT_COMPILE_COMMANDS=Yes",
    "-DBUILD_TESTS=No",
]

ARGS = ["--merge-install", "--symlink-install", "--packages-ignore",
        *IGNORE_PACKAGES, "--cmake-args", *CMAKE_ARGS]


def main():
    # 先编mtc，这个很占内存需要单独编
    env = os.environ.copy()
    env["MAKEFLAGS"] = "-j 8"
    status = subprocess.run(["colcon", "build", "--parallel-workers", "1",
                            "--packages-select", *MTC_PACKAGES, *ARGS], env=env)
    if status.returncode != 0:
        return status.returncode
    # 然后编剩下的包
    status = subprocess.run(
        ["colcon", "build", "--packages-skip", *MTC_PACKAGES, *ARGS], env=os.environ.copy())
    if status.returncode != 0:
        return status.returncode


if __name__ == '__main__':
    sys.exit(main())
