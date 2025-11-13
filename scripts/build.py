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
    "-DBUILD_TESTING=No"
]

ARGS = ["--merge-install", "--symlink-install", "--cmake-force-configure",
    "--packages-ignore", *IGNORE_PACKAGES, "--cmake-args", *CMAKE_ARGS]

def main():
    # 先编mtc,这个很占内存需要单独编
    env = os.environ.copy()
    env["MAKEFLAGS"] = "-j 4"
    status = subprocess.run(["colcon", "build", "--parallel-workers", "1", "--build-base", "build_mtc", "--install-base", "install_mtc",
                            "--packages-select", *MTC_PACKAGES, *ARGS], env=env)
    if status.returncode != 0:
        return status.returncode
    
    # source编译好的mtc
    workspace_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    mtc_setup = os.path.join(workspace_root, "install_mtc", "setup.bash")
    env_for_build = os.environ.copy()
    # 通过bash执行source命令并导出环境变量
    result = subprocess.run(
        f'bash -c "source {mtc_setup} && env"',
        shell=True,
        capture_output=True,
        text=True
    )
    if result.returncode == 0:
        for line in result.stdout.splitlines():
            if '=' in line:
                key, value = line.split('=', 1)
                env_for_build[key] = value
    
    # 然后编剩下的包
    status = subprocess.run(
        ["colcon", "build", "--packages-skip", *MTC_PACKAGES, *ARGS], env=env_for_build)
    if status.returncode != 0:
        return status.returncode


if __name__ == '__main__':
    sys.exit(main())
