"""Acceptance guard behavior and server-only renderer evidence."""
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('doctor', ROOT / 'scripts/doctor.py')
doctor = importlib.util.module_from_spec(spec)
spec.loader.exec_module(doctor)


def test_renderer_requires_actual_gl_renderer_evidence(tmp_path):
    server = tmp_path / 'ogre2.log'
    server.write_text('OpenGL 3+ Rendering Subsystem created.\nGL_VENDOR = Mesa\n')
    assert doctor.renderer(server) is None
    server.write_text('GL_VENDOR = Mesa\nGL_RENDERER = llvmpipe (LLVM 20)\nGL_VERSION = 4.5\n')
    assert 'GL_RENDERER = llvmpipe' in doctor.renderer(server)
    assert doctor.renderer(tmp_path / 'missing.log') is None


@pytest.mark.parametrize('image,platform,accepted', [
    ('registry.example/sim:latest', 'linux/amd64', False),
    ('registry.example/sim@sha256:broken', 'linux/amd64', False),
    ('registry.example/sim@sha256:' + 'a' * 64, 'linux/386', False),
    ('registry.example/sim@sha256:' + 'a' * 64, 'linux/arm64', True),
])
@pytest.mark.parametrize('service', ['sim', 'companion'])
def test_release_rejects_unpinned_or_unsupported_images_before_start(tmp_path, image, platform, accepted, service):
    log = tmp_path / 'calls.jsonl'
    docker = tmp_path / 'docker'
    config = {'services': {name: {'image': 'registry.example/image@sha256:' + 'a' * 64,
                                  'platform': 'linux/amd64'} for name in ['sim', 'companion']}}
    config['services'][service] = {'image': image, 'platform': platform}
    docker.write_text(f'#!{sys.executable}\n' + f'''
import json, sys
with open({str(log)!r}, 'a') as log:
    log.write(json.dumps(sys.argv[1:]) + '\\n')
if 'config' in sys.argv:
    print({json.dumps(config)!r})
''')
    docker.chmod(0o755)
    result = subprocess.run([str(ROOT / 'scripts/release'), 'up', 'sim'], cwd=tmp_path,
                            env={**os.environ, 'PATH': f'{tmp_path}:{os.environ["PATH"]}'},
                            capture_output=True, text=True)
    calls = [json.loads(line) for line in log.read_text().splitlines()]
    assert (result.returncode == 0) == accepted, result.stderr
    assert any('up' in call for call in calls) == accepted


def test_renderer_cannot_certify_a_stopped_server(monkeypatch, capsys):
    monkeypatch.setattr(doctor, 'writable', lambda _: True)
    monkeypatch.setattr(doctor, 'device_checks', lambda _: True)
    monkeypatch.setattr(Path, 'read_text', lambda *a, **k: (_ for _ in ()).throw(OSError('no live server')))
    monkeypatch.setattr(doctor, 'renderer', lambda _: 'GL_RENDERER = hardware GPU')
    assert doctor.container_checks(require_renderer=True) is False
    assert 'UNVALIDATED' in capsys.readouterr().out


@pytest.mark.parametrize('command,expected', [
    (b'gz\0sim\0-s\0/workspace/runs/run1/world.sdf\0', True),
    (b'gz sim --headless-rendering -s /workspace/runs/run1/world.sdf\0\0\0', True),
    (b'gz sim -s /workspace/runs/run1/world.sdf.old\0', False),
    (b'gz sim -s /workspace/runs/other/world.sdf\0', False),
])
def test_live_server_accepts_gazebo_process_title_and_normal_argv(command, expected):
    assert doctor.server_command_uses_world(command, Path('/workspace/runs/run1/world.sdf')) == expected
