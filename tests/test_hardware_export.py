"""hardware_mpc must stay a verbatim, self-contained export of solo_mpc."""

import importlib.util
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("export_hardware", ROOT / "scripts/export_hardware.py")
export = importlib.util.module_from_spec(spec)
spec.loader.exec_module(export)


def test_hardware_package_matches_simulation_sources():
    result = subprocess.run([sys.executable, str(ROOT / "scripts/export_hardware.py"), "--check"],
                            capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + "\nRun: python3 scripts/export_hardware.py"


def test_provenance_hashes_every_exported_module():
    modules = json.loads((ROOT / "hardware_mpc/PROVENANCE.json").read_text())["modules"]
    assert set(modules) == {str(m) for m in export.closure()}
    for name, entry in modules.items():
        assert entry["sha256"] == export.sha256((export.DEST / name).read_bytes()), name


def test_hardware_package_imports_without_the_simulation_tree():
    package = ROOT / "hardware_mpc/src/solo_mpc_hardware"
    code = ("import sys\n"
            "from solo_mpc_hardware.settings import load_settings\n"
            "load_settings('config/scenario.example.yaml', 'config/controller.example.yaml')\n"
            "import solo_mpc_hardware.core.mapping, solo_mpc_hardware.core.planner_jps\n"
            "leaked = [m for m in sys.modules if m.split('.')[0] in ('solo_mpc', 'solo_bringup')]\n"
            "assert not leaked, leaked\n")
    result = subprocess.run([sys.executable, "-I", "-c", f"import sys; sys.path.insert(0, '.')\n{code}"],
                            cwd=package, capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
