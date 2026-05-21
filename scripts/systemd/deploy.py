#!/usr/bin/env python3

"""
Deploy or remove udev + modules-load configuration for can0
"""

import argparse
import os
import shutil
import subprocess
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).parent.resolve()

FILES = [
    (SCRIPT_DIR / "80-can0.rules",    Path("/etc/udev/rules.d/80-can0.rules")),
    (SCRIPT_DIR / "80-fdilink.rules", Path("/etc/udev/rules.d/80-fdilink.rules")),
    (SCRIPT_DIR / "can.conf",         Path("/etc/modules-load.d/can.conf")),
]


def require_root():
    if os.geteuid() != 0:
        print("error: this script must be run as root (use sudo)", file=sys.stderr)
        sys.exit(1)


def reload_udev():
    subprocess.run(["udevadm", "control", "--reload-rules"], check=True)
    print("udev rules reloaded.")
    # 立即触发规则
    subprocess.run(
        ["udevadm", "trigger"],
        check=True,
    )
    print("udev trigger sent.")


def install():
    require_root()
    for src, dst in FILES:
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src, dst)
        print(f"installed: {dst}")
    reload_udev()


def uninstall():
    require_root()
    for _, dst in FILES:
        if dst.exists():
            dst.unlink()
            print(f"removed:   {dst}")
        else:
            print(f"not found: {dst} (skipped)")
    reload_udev()


def main():
    parser = argparse.ArgumentParser()
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--install",   action="store_true", help="install configuration files")
    group.add_argument("--uninstall", action="store_true", help="remove configuration files")
    args = parser.parse_args()

    if args.install:
        install()
    else:
        uninstall()


if __name__ == "__main__":
    main()
