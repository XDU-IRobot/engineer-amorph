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
    # # rm_referee_ros2
    # "rm_referee",
    # "rm_referee_msgs",
    # # sbus_ros2
    # "sbus_bridge",
    # "sbus_interface",
    # easy_handeye2
    "easy_handeye2",
    "easy_handeye2_msgs",
    # orbbec sdk
    "orbbec_camera",
    "orbbec_camera_msgs",
    "orbbec_description",
    "fdilink_ahrs",
]

CMAKE_ARGS = [
    "-DCMAKE_BUILD_TYPE=RelWithDebInfo",
    "-DCMAKE_EXPORT_COMPILE_COMMANDS=Yes",
    "-DBUILD_TESTS=No",
    "-DBUILD_TESTING=No"
]


def assert_retval(status):
    if status.returncode != 0:
        sys.exit(status.returncode)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-j', '--jobs', type=int, default=12,
                        help='Number of parallel jobs for mtc compilation (default: 12)')
    parser.add_argument('-f', '--fast', action='store_true',
                        help='Parallel build third-party packages.')
    parser.add_argument('-c', '--cmake-configure', action='store_true',
                        help='Force CMake configure step.')
    args = parser.parse_args()
    
    if args.cmake_configure:
        ARGS = ["--merge-install", "--symlink-install", "--cmake-force-configure",
            "--packages-ignore", *IGNORE_PACKAGES, "--cmake-args", *CMAKE_ARGS]
    else:
        ARGS = ["--merge-install", "--symlink-install",
            "--packages-ignore", *IGNORE_PACKAGES, "--cmake-args", *CMAKE_ARGS]

    # Use sys.executable to run colcon module, ensuring we stay in the current python environment (venv)
    colcon_cmd = [sys.executable, "-m", "colcon"]

    # 先编第三方的包，这些包不会有任何改动，所以单独编好放在另外的build目录不再动
    env = os.environ.copy()
    if args.fast:
        # 所有包一起编，适用于已经编译好需要快速过掉第一阶段编译thirdparty的时候
        assert_retval(subprocess.run(colcon_cmd + ["build", "--build-base", "build_3rdparty",
                                                   "--packages-select", *THIRDPARTY_PACKAGES, *ARGS], env=env))
    else:
        # 先编mtc，这个编译起来巨tm占内存，所以限制只能单线程编译
        env["MAKEFLAGS"] = f"-j {args.jobs}"      # 限制并行数
        MTC_PACKAGES = [s for s in THIRDPARTY_PACKAGES if s.startswith(
            "moveit_task_constructor")] + ["rviz_marker_tools"]
        assert_retval(subprocess.run(colcon_cmd + ["build", "--parallel-workers", "1", "--build-base", "build_3rdparty",
                                                   "--packages-select", *MTC_PACKAGES, *ARGS], env=env))
        # 然后编剩下的包
        env["MAKEFLAGS"] = f"-j {os.cpu_count()}"  # 恢复默认最大并行数
        REMAIN_PACKAGES = [
            s for s in THIRDPARTY_PACKAGES if s not in MTC_PACKAGES]
        assert_retval(subprocess.run(colcon_cmd + ["build", "--build-base", "build_3rdparty",
                                                   "--packages-select", *REMAIN_PACKAGES, *ARGS], env=env))

    # source编译好的thirdparty packages
    workspace_root = os.path.dirname(
        os.path.dirname(os.path.abspath(__file__)))
    thirdparty_setup = os.path.join(workspace_root, "install", "setup.bash")
    env_for_build = os.environ.copy()
    # 通过bash执行source命令并导出环境变量
    result = subprocess.run(
        f'bash -c "source {thirdparty_setup} && env"',
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
    assert_retval(subprocess.run(
        colcon_cmd + ["build", "--cmake-force-configure", "--packages-skip", *THIRDPARTY_PACKAGES, *ARGS], env=env_for_build))


if __name__ == '__main__':
    sys.exit(main())
