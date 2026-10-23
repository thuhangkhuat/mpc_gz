"""Check mission launch wiring and shutdown order without starting vehicles."""
import json
import os
from pathlib import Path
import subprocess
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def fake_docker(tmp_path):
    log = tmp_path / "docker.jsonl"
    docker = tmp_path / "docker"
    docker.write_text(f"#!{sys.executable}\n" + '''import json, os, sys
from pathlib import Path
args = sys.argv[1:]
with open(os.environ["DOCKER_TEST_LOG"], "a") as log:
    log.write(json.dumps({"args": args, "env": {k: os.environ.get(k) for k in (
        "FLEET_CONFIG", "MPC_SCENARIO", "MPC_SETTINGS", "MPC_TARGET_MODE", "MPC_RECORD_CAMERAS",
        "MPC_RECORD_OVERHEAD", "RUN_ID", "COMPOSE_FILE", "COMPOSE_PATH_SEPARATOR",
        "LIBGL_ALWAYS_SOFTWARE")}}) + "\\n")
if args == ["compose", "config", "--environment"]:
    print(os.environ.get("TEST_COMPOSE_ENV", ""))
if args[0] == "inspect":
    print('"sha256:test"')
elif "ps" in args and "-q" in args and "--status" not in args:
    print("test-container")
elif args[-2:] == ["printenv", "READINESS_TIMEOUT"]:
    print("600")
if args[-2:] == ["mission", "stop"] and os.environ.get("FAIL_MISSION_STOP"):
    sys.exit(1)
''')
    docker.chmod(0o755)
    python = tmp_path / 'python3'
    python.write_text(f'#!{sys.executable}\n' + '''import json, os, sys
if sys.argv[1:2] == ["scripts/wait_ready.py"]:
    with open(os.environ["DOCKER_TEST_LOG"], "a") as log:
        log.write(json.dumps({"args": sys.argv[1:], "env": {"RUN_ID": os.environ.get("RUN_ID")}}) + "\\n")
    sys.exit(int(os.environ.get("FAIL_READINESS", "0")))
os.execv(sys.executable, [sys.executable, *sys.argv[1:]])
''')
    python.chmod(0o755)
    env = {**os.environ, "PATH": f"{tmp_path}:{os.environ['PATH']}", "DOCKER_TEST_LOG": str(log),
           "COMPOSE_FILE": "compose.yaml"}
    return log, env


def invoke(args, env):
    return subprocess.run([str(ROOT / "scripts/dev"), *args], env=env,
                          capture_output=True, text=True, timeout=10)


def calls(log):
    return [json.loads(line) for line in log.read_text().splitlines()] if log.exists() else []


def test_dotenv_gpu_selection_reaches_compose(fake_docker, tmp_path):
    log, env = fake_docker
    env.pop("COMPOSE_FILE")
    env.pop("COMPOSE_PATH_SEPARATOR", None)
    # Use an isolated checkout so the test does not depend on the user's .env.
    project = tmp_path / "project"
    (project / "scripts").mkdir(parents=True)
    launcher = project / "scripts/dev"
    launcher.write_bytes((ROOT / "scripts/dev").read_bytes())
    launcher.chmod(0o755)
    (project / ".env").write_text('COMPOSE_FILE="compose.yaml;compose.nvidia.yaml"\n')
    env["TEST_COMPOSE_ENV"] = (
        "COMPOSE_FILE=compose.yaml;compose.nvidia.yaml\nCOMPOSE_PATH_SEPARATOR=;"
    )
    result = subprocess.run([str(launcher), "pull"], env=env, capture_output=True,
                            text=True, timeout=10)
    assert result.returncode == 0, result.stderr
    entries = calls(log)
    assert entries[0]["args"] == ["compose", "config", "--environment"]
    selected = entries[-1]
    assert "-f" not in selected["args"]
    assert selected["env"]["COMPOSE_FILE"].split(";")[:2] == [
        "compose.yaml", "compose.nvidia.yaml"
    ]
    assert selected["env"]["COMPOSE_PATH_SEPARATOR"] == ";"


def test_explicit_shell_gpu_and_rendering_overrides_are_preserved(fake_docker):
    log, env = fake_docker
    env["COMPOSE_FILE"] = "compose.yaml:compose.nvidia.yaml"
    env["LIBGL_ALWAYS_SOFTWARE"] = "1"
    result = invoke(["pull"], env)
    assert result.returncode == 0, result.stderr
    entries = calls(log)
    assert len(entries) == 1
    assert "-f" not in entries[0]["args"]
    assert entries[0]["env"]["COMPOSE_FILE"].startswith("compose.yaml:compose.nvidia.yaml")
    assert entries[0]["env"]["LIBGL_ALWAYS_SOFTWARE"] == "1"


@pytest.mark.parametrize("args", [["sim"], ["flight", "takeoff"], ["flight", "position"],
                                  ["mpc", "--profile", "control"], ["mpc", "--resume", "old"]])
def test_legacy_commands_are_rejected_before_docker(fake_docker, args):
    log, env = fake_docker
    assert invoke(args, env).returncode == 2
    assert calls(log) == []


def test_mpc_passes_custom_configuration_to_compose(fake_docker):
    log, env = fake_docker
    result = invoke(["mpc", "--no-wait", "--fleet", "/workspace/config/custom-fleet.yaml",
                     "--scenario", "/workspace/custom-scenario.yaml", "--settings",
                     "/workspace/custom-settings.yaml"], env)
    entries = calls(log)
    try:
        assert result.returncode == 0, result.stderr
        start = next(e for e in entries if "up" in e["args"])
        assert start["env"]["FLEET_CONFIG"] == "/workspace/config/custom-fleet.yaml"
        assert start["env"]["MPC_SCENARIO"] == "/workspace/custom-scenario.yaml"
        assert start["env"]["MPC_SETTINGS"] == "/workspace/custom-settings.yaml"
    finally:
        for entry in entries:
            if entry["env"]["RUN_ID"]:
                (ROOT / "runs" / (entry["env"]["RUN_ID"] + ".image-id.json")).unlink(missing_ok=True)


@pytest.mark.parametrize("record", [False, True])
@pytest.mark.parametrize("overhead", [False, True])
def test_camera_recording_is_opt_in(fake_docker, record, overhead):
    log, env = fake_docker
    env["MPC_RECORD_CAMERAS"] = "true"
    env["MPC_RECORD_OVERHEAD"] = "true"
    args = ["mpc", "--no-wait"] + (["--record-cameras"] if record else [])
    if overhead:
        args.append("--record-overhead")
    result = invoke(args, env)
    entries = calls(log)
    try:
        assert result.returncode == 0, result.stderr
        start = next(e for e in entries if "up" in e["args"])
        assert start["env"]["MPC_RECORD_CAMERAS"] == str(record).lower()
        assert start["env"]["MPC_RECORD_OVERHEAD"] == str(overhead).lower()
    finally:
        for entry in entries:
            if entry["env"]["RUN_ID"]:
                (ROOT / "runs" / (entry["env"]["RUN_ID"] + ".image-id.json")).unlink(missing_ok=True)


@pytest.mark.parametrize('configured_mode', ['', 'camera', 'ground_truth'])
@pytest.mark.parametrize('ground_truth', [False, True])
def test_ground_truth_flag_overrides_configured_source(fake_docker, configured_mode, ground_truth):
    log, env = fake_docker
    env['MPC_TARGET_MODE'] = configured_mode
    result = invoke(['mpc', '--no-wait'] + (['--ground-truth-target'] if ground_truth else []), env)
    entries = calls(log)
    try:
        assert result.returncode == 0, result.stderr
        start = next(e for e in entries if 'up' in e['args'])
        assert start['env']['MPC_TARGET_MODE'] == ('ground_truth' if ground_truth else configured_mode)
    finally:
        for entry in entries:
            if entry['env']['RUN_ID']:
                (ROOT / 'runs' / (entry['env']['RUN_ID'] + '.image-id.json')).unlink(missing_ok=True)


def test_stop_lands_mission_before_stopping_container(fake_docker):
    log, env = fake_docker
    assert invoke(["stop"], env).returncode == 0
    commands = [e["args"] for e in calls(log)]
    assert len(commands) == 2
    assert commands[0][-4:] == ["run", "solo_mpc", "mission", "stop"]
    assert commands[1][-2:] == ["stop", "sim"]


def test_failed_landing_keeps_container_running(fake_docker):
    log, env = fake_docker
    env["FAIL_MISSION_STOP"] = "1"
    assert invoke(["stop"], env).returncode == 1
    assert len(calls(log)) == 1


def test_force_stop_only_stops_processes(fake_docker):
    log, env = fake_docker
    assert invoke(["stop", "--force"], env).returncode == 0
    commands = [e["args"] for e in calls(log)]
    assert len(commands) == 1
    assert commands[0][-2:] == ["stop", "sim"]


@pytest.mark.parametrize("args", [["gui"], ["mpc", "--no-wait", "--gui"]])
def test_gui_commands_start_target_follow_helper(fake_docker, args):
    log, env = fake_docker
    result = invoke(args, env)
    entries = calls(log)
    try:
        assert result.returncode == 0, result.stderr
        assert any(e['args'][-4:] == [
            "sim", "/usr/local/bin/solo-entrypoint", "python3", "/workspace/scripts/gazebo_gui.py"
        ] for e in entries)
        if args[0] == 'mpc':
            assert not any('mission' in e['args'] for e in entries)
    finally:
        for entry in entries:
            if entry["env"]["RUN_ID"]:
                (ROOT / "runs" / (entry["env"]["RUN_ID"] + ".image-id.json")).unlink(missing_ok=True)
                (ROOT / 'runs' / (entry['env']['RUN_ID'] + '.gui.log')).unlink(missing_ok=True)


@pytest.mark.parametrize('no_wait', [False, True])
def test_mpc_starts_stack_without_starting_mission(fake_docker, no_wait):
    log, env = fake_docker
    result = invoke(['mpc'] + (['--no-wait'] if no_wait else []), env)
    entries = calls(log)
    try:
        assert result.returncode == 0, result.stderr
        commands = [e['args'] for e in entries]
        assert any('up' in c for c in commands)
        assert not any('mission' in c for c in commands)
        waits = [c for c in commands if c[0] == 'scripts/wait_ready.py']
        assert bool(waits) == (not no_wait)
        if waits:
            assert '--all-processes' in waits[0]
    finally:
        for entry in entries:
            if entry['env']['RUN_ID']:
                (ROOT / 'runs' / (entry['env']['RUN_ID'] + '.image-id.json')).unlink(missing_ok=True)


def test_mpc_does_not_start_mission_after_readiness_failure(fake_docker):
    log, env = fake_docker
    env['FAIL_READINESS'] = '1'
    result = invoke(['mpc'], env)
    entries = calls(log)
    try:
        assert result.returncode == 1
        assert not any('mission' in e['args'] for e in entries)
    finally:
        for entry in entries:
            if entry['env']['RUN_ID']:
                (ROOT / 'runs' / (entry['env']['RUN_ID'] + '.image-id.json')).unlink(missing_ok=True)


@pytest.mark.parametrize('seconds', ['20', '-1', 'nan', 'inf', 'bad'])
def test_mpc_rejects_mission_duration_before_starting_stack(fake_docker, seconds):
    log, env = fake_docker
    assert invoke(['mpc', '--seconds', seconds], env).returncode != 0
    assert calls(log) == []


def test_mission_run_is_an_explicit_separate_command(fake_docker):
    log, env = fake_docker
    result = invoke(['mission', 'run', '--seconds', '20'], env)
    assert result.returncode == 0, result.stderr
    commands = [e['args'] for e in calls(log)]
    assert len(commands) == 1
    assert commands[0][-6:] == ['run', 'solo_mpc', 'mission', 'run', '--seconds', '20']
