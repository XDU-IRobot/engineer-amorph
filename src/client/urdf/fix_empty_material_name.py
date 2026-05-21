#!/usr/bin/env python3
"""
Fix URDF/xacro files missing the required 'name' attribute on <material> tags.

URDF spec requires <material name="..."> inside <visual> elements.
This script adds name="" to any <material> tag that lacks it.
"""

import argparse
import re
import sys
from pathlib import Path

# Matches <material> or <material > without any name= attribute
_MATERIAL_NO_NAME_RE = re.compile(r"<material\s*>")


def fix_file(filepath: Path, dry_run: bool = False) -> bool:
    """Fix a single file. Returns True if changes were made."""
    content = filepath.read_text()
    new_content, count = _MATERIAL_NO_NAME_RE.subn("<material name=\"\">", content)
    if count == 0:
        return False

    if dry_run:
        print(f"[dry-run] {filepath}: {count} tag(s) would be fixed")
        return True

    filepath.write_text(new_content)
    print(f"[fixed] {filepath}: {count} tag(s)")
    return True


def collect_files(paths: list[Path]) -> list[Path]:
    """Collect .xacro/.urdf files from given paths (files or directories)."""
    files: list[Path] = []
    for p in paths:
        if p.is_file():
            if p.suffix in (".xacro", ".urdf"):
                files.append(p)
        elif p.is_dir():
            for ext in ("*.xacro", "*.urdf"):
                files.extend(p.rglob(ext))
    return sorted(set(files))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "paths", nargs="+", type=Path,
        help="URDF/xacro files or directories to scan",
    )
    parser.add_argument(
        "--dry-run", action="store_true",
        help="Report what would be changed without modifying files",
    )
    args = parser.parse_args()

    files = collect_files(args.paths)
    if not files:
        print("No .xacro or .urdf files found.", file=sys.stderr)
        sys.exit(1)

    changed = 0
    for f in files:
        if fix_file(f, dry_run=args.dry_run):
            changed += 1

    verb = "would be" if args.dry_run else ""
    print(f"\n{changed} file(s) {verb} updated." if not args.dry_run else f"\n{changed} file(s) would be updated.")


if __name__ == "__main__":
    main()
