#!/usr/bin/env python3

import os
import sys
import subprocess
import argparse

IGNORE_PACKAGES = [
    "moveit_task_constructor_demo",
]
THIRDPARTY_PACKAGES = [
    # MTC
    "rviz_marker_tools",
    "moveit_task_constructor_core",
    "moveit_task_constructor_msgs",
    "moveit_task_constructor_capabilities",
    "moveit_task_constructor_visualization",
    # rm2
    "referee_node",
    "rm2_referee_msgs",
    "rm2_remote",
    # sbus_ros2
    "sbus_bridge",
    "sbus_interface",
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
    parser = argparse.ArgumentParser(description='Build ROS 2 workspace with MTC')
    parser.add_argument('-j', '--jobs', type=int, default=12,
                        help='Number of parallel jobs for thirdparty compilation (default: 12)')
    parser.add_argument('-s', '--skip-thirdparty', action='store_true',
                        help='Skip building thirdparty packages (assumes they are already built)')
    args = parser.parse_args()
    
    # Use sys.executable to run colcon module, ensuring we stay in the current python environment (venv)
    colcon_cmd = [sys.executable, "-m", "colcon"]
    
    # 先编第三方的包，这些包不会有任何改动，所以单独编好放在另外的build和install目录不再动
    if not args.skip_thirdparty:
        env = os.environ.copy()
        env["MAKEFLAGS"] = f"-j {args.jobs}"      # 并行数太多内存会爆

        status = subprocess.run(colcon_cmd + ["build", "--parallel-workers", "1", "--build-base", "build_mtc", "--install-base", "install_mtc",
                                "--packages-select", *THIRDPARTY_PACKAGES, *ARGS], env=env)
        if status.returncode != 0:
            return status.returncode
    else:
        print("Skipping thirdparty packages compilation (--skip-thirdparty flag set)")
    
    # source编译好的thirdparty packages
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
        colcon_cmd + ["build", "--cmake-force-configure", "--packages-skip", *THIRDPARTY_PACKAGES, *ARGS], env=env_for_build)
    if status.returncode != 0:
        return status.returncode


if __name__ == '__main__':
    sys.exit(main())
