#!/usr/bin/env python3

import os
import sys
import subprocess
import argparse
import shutil

# Onboard (Jetson) 专用的编译包列表
ONBOARD_PACKAGES = [
    "amorph_assets",
    "amorph_bringup",
    "amorph_vision",
    "amorph_onboard_transport",
    "orbbec_camera",
    "orbbec_camera_msgs",
    "orbbec_description",
]

CMAKE_ARGS = [
    "-DCMAKE_BUILD_TYPE=Release",
    "-DCMAKE_CXX_FLAGS=-Wno-error=stringop-overflow -Wno-error",
]


def find_venv(workspace_root: str) -> str | None:
    """查找可用的 venv 路径"""
    candidates = [
        os.path.join(workspace_root, ".venv"),
        os.path.join(workspace_root, "venv"),
    ]
    for path in candidates:
        activate = os.path.join(path, "bin", "activate")
        if os.path.isfile(activate):
            return path
    return None


def activate_venv(venv_path: str, env: dict) -> dict:
    """在环境变量中激活 venv"""
    venv_bin = os.path.join(venv_path, "bin")
    env["VIRTUAL_ENV"] = venv_path
    env["PATH"] = f"{venv_bin}{os.pathsep}{env.get('PATH', '')}"
    # 清除可能导致问题的 Python 相关环境变量
    env.pop("PYTHONHOME", None)
    return env


def source_ros_setup(env: dict) -> dict:
    """source ROS 2 setup.bash 并将环境变量合并到 env 中"""
    ros_distro = os.environ.get("ROS_DISTRO", "humble")
    ros_setup = f"/opt/ros/{ros_distro}/setup.bash"

    if not os.path.isfile(ros_setup):
        print(f"[WARN] ROS 2 setup not found at {ros_setup}, skipping ROS source", file=sys.stderr)
        return env

    result = subprocess.run(
        f'bash -c "source {ros_setup} && env"',
        shell=True,
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        print(f"[ERROR] Failed to source ROS 2 setup: {ros_setup}", file=sys.stderr)
        sys.exit(1)

    for line in result.stdout.splitlines():
        if "=" in line:
            key, value = line.split("=", 1)
            env[key] = value

    return env


def assert_retval(status: subprocess.CompletedProcess):
    if status.returncode != 0:
        sys.exit(status.returncode)


def main():
    parser = argparse.ArgumentParser(
        description="Onboard (Jetson) 编译脚本 —— 支持 venv 环境"
    )
    parser.add_argument(
        "--venv",
        type=str,
        default=None,
        help="指定 venv 路径（默认自动查找工作空间下的 .venv 或 venv）",
    )
    parser.add_argument(
        "--no-venv",
        action="store_true",
        help="不使用 venv 环境",
    )
    parser.add_argument(
        "--no-ros",
        action="store_true",
        help="不 source ROS 2 环境",
    )
    parser.add_argument(
        "-j", "--jobs",
        type=int,
        default=None,
        help="并行编译任务数（默认使用 CPU 核心数）",
    )
    parser.add_argument(
        "--cmake-force-configure",
        action="store_true",
        help="强制重新执行 CMake configure",
    )
    parser.add_argument(
        "--packages-select",
        type=str,
        nargs="+",
        default=None,
        help="覆盖默认的包列表，指定要编译的包",
    )
    parser.add_argument(
        "--cmake-args",
        type=str,
        nargs="+",
        default=None,
        help="额外的 CMake 参数",
    )
    args = parser.parse_args()

    workspace_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    env = os.environ.copy()

    # 1. Source ROS 2 环境
    if not args.no_ros:
        print("[INFO] Sourcing ROS 2 setup...")
        env = source_ros_setup(env)

    # 2. 激活 venv
    if not args.no_venv:
        venv_path = args.venv
        if venv_path is None:
            venv_path = find_venv(workspace_root)
        if venv_path is not None:
            print(f"[INFO] Activating venv: {venv_path}")
            env = activate_venv(venv_path, env)
        else:
            print("[WARN] No venv found, using system Python", file=sys.stderr)

    # 3. 使用 venv 中的 colcon（如果 venv 已激活），否则使用系统 colcon
    if not args.no_venv and "VIRTUAL_ENV" in env:
        colcon_exe = os.path.join(env["VIRTUAL_ENV"], "bin", "colcon")
        if os.path.isfile(colcon_exe):
            colcon_cmd = [colcon_exe]
        else:
            colcon_cmd = [sys.executable, "-m", "colcon"]
    else:
        colcon_cmd = ["colcon"]

    # 检查 colcon 是否可用
    if shutil.which(colcon_cmd[0]) is None and colcon_cmd[0] != sys.executable:
        print(f"[ERROR] colcon not found: {colcon_cmd[0]}", file=sys.stderr)
        sys.exit(1)

    # 4. 构建参数
    build_args = ["--merge-install", "--symlink-install"]

    if args.cmake_force_configure:
        build_args.append("--cmake-force-configure")

    packages = args.packages_select if args.packages_select else ONBOARD_PACKAGES
    build_args.extend(["--packages-select", *packages])

    # CMAKE 参数
    cmake_args = list(CMAKE_ARGS)
    if args.cmake_args:
        cmake_args.extend(args.cmake_args)
    build_args.extend(["--cmake-args", *cmake_args])

    # 并行任务数
    if args.jobs is not None:
        env["MAKEFLAGS"] = f"-j{args.jobs}"

    # 5. 执行编译
    print(f"[INFO] Building packages: {packages}")
    print(f"[INFO] Command: {' '.join(colcon_cmd + ['build'] + build_args)}")
    print("=" * 60)

    result = subprocess.run(
        colcon_cmd + ["build"] + build_args,
        env=env,
        cwd=workspace_root,
    )
    assert_retval(result)

    print("[INFO] Build completed successfully!")


if __name__ == "__main__":
    sys.exit(main())
