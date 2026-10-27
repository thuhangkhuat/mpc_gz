"""Regenerate hardware_mpc's Python package from the simulation's solo_mpc package.

The hardware package is a verbatim copy of the modules the controller and the
optional detector import (found by walking relative imports), so simulation
fixes reach the Pi without hand edits. Run after changing solo_mpc:

  python3 scripts/export_hardware.py           # rewrite hardware_mpc (+ solo_ros)
  python3 scripts/export_hardware.py --check   # exit 1 if either copy is stale

Packaging, launch, configuration, docs and locks under hardware_mpc are
hand-maintained and left alone. PROVENANCE.json records the source commit and
the SHA-256 of every exported module.

The MPC runs inside the Pi's `solo` container, so the whole ament package and
the numerical lock are then mirrored into the solo_ros tree
(../3DR/solo_ros by default, skipped when absent; --solo-ros overrides).
"""

import argparse
import ast
import hashlib
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "ros_ws/src/solo_mpc/solo_mpc"
PACKAGE = "solo_mpc_hardware"
DEST = ROOT / "hardware_mpc/src" / PACKAGE / PACKAGE
PROVENANCE = ROOT / "hardware_mpc/PROVENANCE.json"
ENTRY_POINTS = ("controller_node.py", "perception_node.py", "bench_solver.py")
PACKAGE_DIR = ROOT / "hardware_mpc/src" / PACKAGE
LOCK = ROOT / "hardware_mpc/requirements/mpc-py310-aarch64.lock"
SOLO_ROS = ROOT.parent / "3DR/solo_ros"


def relative_imports(path):
    """Package-relative module paths imported by one source file."""
    tree = ast.parse(path.read_text(), str(path))
    package = path.parent.relative_to(SOURCE).parts
    found = set()
    for node in ast.walk(tree):
        if not isinstance(node, ast.ImportFrom) or node.level == 0:
            continue
        base = list(package[:len(package) - (node.level - 1)]) if node.level > 1 else list(package)
        parts = base + (node.module.split(".") if node.module else [])
        candidates = [parts] if node.module else []
        # `from . import x` / `from .core import y` may name submodules.
        candidates += [parts + [alias.name] for alias in node.names]
        for candidate in candidates:
            for target in (SOURCE.joinpath(*candidate).with_suffix(".py"),
                           SOURCE.joinpath(*candidate, "__init__.py")):
                if target.is_file():
                    found.add(target.relative_to(SOURCE))
    return found


def closure():
    modules, pending = set(), [Path(p) for p in ENTRY_POINTS]
    while pending:
        module = pending.pop()
        if module in modules:
            continue
        modules.add(module)
        pending.extend(relative_imports(SOURCE / module) - modules)
    # Every package directory on the way needs its __init__.py.
    for module in list(modules):
        for parent in module.parents:
            if str(parent) != "." and (SOURCE / parent / "__init__.py").is_file():
                modules.add(parent / "__init__.py")
    modules.add(Path("__init__.py"))
    return sorted(modules)


def sha256(data):
    return hashlib.sha256(data).hexdigest()


def git(*args):
    try:
        return subprocess.check_output(["git", *args], cwd=ROOT, text=True).strip()
    except (OSError, subprocess.CalledProcessError):
        return None


def expected():
    files = {str(m): (SOURCE / m).read_bytes() for m in closure()}
    for name, data in files.items():
        if b"solo_mpc." in data or b"import solo_mpc" in data:
            raise SystemExit(f"{name} imports solo_mpc absolutely; use a relative import")
    return files


def provenance(files):
    dirty = git("status", "--porcelain", "--", str(SOURCE.relative_to(ROOT)))
    return {
        "source_package": str(SOURCE.parent.relative_to(ROOT)),
        "source_commit": git("rev-parse", "HEAD"),
        "source_dirty": bool(dirty),
        "package": PACKAGE,
        "generator": "scripts/export_hardware.py",
        "changes": [
            f"Verbatim copy of the modules reachable from {', '.join(ENTRY_POINTS)}; "
            f"only the ROS/Python package name differs ({PACKAGE}).",
        ],
        "modules": {name: {"source": str((SOURCE / name).relative_to(ROOT)), "sha256": sha256(data)}
                    for name, data in files.items()},
    }


def stale(files):
    present = {str(p.relative_to(DEST)) for p in DEST.rglob("*.py")} if DEST.exists() else set()
    changed = sorted(n for n, d in files.items() if not (DEST / n).is_file() or (DEST / n).read_bytes() != d)
    return changed, sorted(present - set(files))


def mirror_files(solo_ros):
    """solo_ros destination path -> bytes for the package tree and the lock."""
    files = {solo_ros / "ros2_ws/src" / PACKAGE / p.relative_to(PACKAGE_DIR): p.read_bytes()
             for p in sorted(PACKAGE_DIR.rglob("*"))
             if p.is_file() and "__pycache__" not in p.parts and p.suffix != ".pyc"}
    files[solo_ros / "docker" / LOCK.name] = LOCK.read_bytes()
    return files


def mirror_stale(solo_ros, files):
    dest = solo_ros / "ros2_ws/src" / PACKAGE
    present = {p for p in dest.rglob("*") if p.is_file() and "__pycache__" not in p.parts
               and p.suffix != ".pyc"} if dest.exists() else set()
    changed = sorted(p for p, d in files.items() if not p.is_file() or p.read_bytes() != d)
    return changed, sorted(present - set(files))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--check", action="store_true", help="report differences instead of writing")
    parser.add_argument("--solo-ros", type=Path, default=SOLO_ROS,
                        help=f"solo_ros tree to mirror into (default {SOLO_ROS}; skipped if missing)")
    args = parser.parse_args(argv)
    files = expected()
    changed, extra = stale(files)
    solo_ros = args.solo_ros if (args.solo_ros / "docker/Dockerfile").is_file() else None
    if args.check:
        for name in changed:
            print(f"stale: {name}")
        for name in extra:
            print(f"not exported (remove): {name}")
        if changed or extra:
            print("hardware_mpc is out of date: run python3 scripts/export_hardware.py")
            return 1
        print(f"hardware_mpc matches solo_mpc ({len(files)} modules)")
        if solo_ros is not None:
            m_changed, m_extra = mirror_stale(solo_ros, mirror_files(solo_ros))
            for path in m_changed + m_extra:
                print(f"solo_ros stale: {path}")
            if m_changed or m_extra:
                print("solo_ros is out of date: run python3 scripts/export_hardware.py")
                return 1
            print(f"solo_ros matches hardware_mpc ({solo_ros})")
        return 0
    for name, data in files.items():
        (DEST / name).parent.mkdir(parents=True, exist_ok=True)
        (DEST / name).write_bytes(data)
    for name in extra:
        (DEST / name).unlink()
    PROVENANCE.write_text(json.dumps(provenance(files), indent=2) + "\n")
    print(f"exported {len(files)} modules: {len(changed)} updated, {len(extra)} removed")
    for name in changed:
        print(f"  updated {name}")
    for name in extra:
        print(f"  removed {name}")
    if solo_ros is None:
        print(f"solo_ros not found at {args.solo_ros}: not mirrored")
        return 0
    mirror = mirror_files(solo_ros)
    m_changed, m_extra = mirror_stale(solo_ros, mirror)
    for path in m_changed:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(mirror[path])
    for path in m_extra:
        path.unlink()
    print(f"mirrored into {solo_ros}: {len(m_changed)} updated, {len(m_extra)} removed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
