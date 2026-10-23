#!/usr/bin/env python3
"""Check the selected Docker service, including its live server renderer."""
import argparse
import glob
import json
import os
from pathlib import Path
import platform
import re
import subprocess
import sys
import tempfile


def writable(path):
    try:
        with tempfile.NamedTemporaryFile(dir=path):
            pass
        print(f'PASS writable: {path} (uid={os.getuid()}, groups={os.getgroups()})')
        return True
    except OSError as error:
        print(f'FAIL writable: {path}: {error}')
        return False


def device_checks(patterns):
    ok = True
    for device in sorted({p for pattern in patterns for p in glob.glob(pattern)}):
        stat = os.stat(device)
        allowed = os.access(device, os.R_OK | os.W_OK)
        print(f'{"PASS" if allowed else "FAIL"} device: {device} '
              f'uid:gid={stat.st_uid}:{stat.st_gid} mode={stat.st_mode & 0o777:o}; '
              f'process groups={os.getgroups()}')
        ok &= allowed
    return ok


def renderer(log):
    """Read only the dedicated Gazebo *server* log, never a GUI/GLX probe."""
    try:
        contents = Path(log).read_text(errors='replace')
    except OSError:
        return None
    values = re.findall(r'\bGL_(?:VENDOR|RENDERER|VERSION)\s*=\s*[^\r\n]+', contents)
    if not any(value.startswith('GL_RENDERER') for value in values):
        return None
    return '; '.join(values)


def server_command_uses_world(command, world):
    # gz's Ruby CLI rewrites argv into one process-title string on Linux.
    # Accept both that form and normal NUL-separated argv, with exact boundaries.
    command = command.decode(errors='replace').replace('\0', ' ')
    return re.search(r'(^|\s)' + re.escape(str(world)) + r'(?=\s|$)', command) is not None


def container_checks(require_renderer=False):
    print(f'Container architecture: {platform.machine()}')
    ok = all([writable('/tmp'), writable('/workspace/runs'), writable('/workspace/artifacts')])
    ok &= device_checks(['/dev/dri/card*', '/dev/dri/renderD*', '/dev/nvidia[0-9]*',
                         '/dev/dxg', '/dev/solo-fcu'])
    log = Path('/workspace/runs/latest/gazebo-home/.gz/rendering/ogre2.log')
    # A prior run's log is evidence only while that exact server is alive.
    try:
        run_dir = log.parents[3].resolve()
        startup = json.loads((run_dir / 'startup.json').read_text())
        server = startup['processes']['gazebo-server']
        command = Path(f"/proc/{server['pid']}/cmdline").read_bytes()
        live = server['state'] == 'running' and server_command_uses_world(command, run_dir / 'world.sdf')
    except (OSError, KeyError, ValueError):
        live = False
    value = renderer(log) if live else None
    if value:
        print(f'PASS Gazebo server renderer ({log.resolve()}): {value}')
        if re.search(r'llvmpipe|softpipe|swrast', value, re.I):
            print('Software rendering is active; GPU acceleration is not validated.')
    else:
        print('UNVALIDATED Gazebo server renderer: start the simulation with rendering '
              'sensors, then rerun doctor against the running container.')
        ok &= not require_renderer
    return ok


def run(args, **kwargs):
    return subprocess.run(args, check=True, text=True, **kwargs)


def host_checks(compose, service, require_renderer=False):
    if service == 'companion':
        compose = [*compose, '--profile', 'hardware']
    config = json.loads(run(['docker', *compose, 'config', '--format', 'json'],
                            capture_output=True).stdout)
    definition = config['services'][service]
    architecture = run(['docker', 'version', '--format', '{{.Server.Arch}}'],
                       capture_output=True).stdout.strip()
    print(f'Host architecture: {platform.machine()}; Docker daemon architecture: {architecture}', flush=True)
    image = definition['image']
    image_arch = run(['docker', 'image', 'inspect', image, '--format', '{{.Architecture}}'],
                     capture_output=True).stdout.strip()
    requested = definition.get('platform', f'linux/{architecture}').split('/')[1]
    print(f'Image: {image}; architecture: {image_arch}; requested: {requested}', flush=True)
    ok = architecture == image_arch == requested
    if not ok:
        print('FAIL architecture mismatch; choose a tested native platform digest.', flush=True)
    for device in definition.get('devices', []):
        source = device['source']
        if Path(source).exists():
            stat = os.stat(source)
            print(f'Host device {source}: gid={stat.st_gid}; configured groups='
                  f'{definition.get("group_add", [])}. Container access is checked below.', flush=True)
        else:
            print(f'Host device {source} absent; verify it on the Docker daemon host.', flush=True)
    for mount in definition.get('volumes', []):
        if mount['type'] == 'bind':
            source = Path(mount['source'])
            print(f'Bind: {source} -> {mount["target"]} '
                  f'({"read-only" if mount.get("read_only") else "writable"})', flush=True)
            if not source.exists():
                print(f'FAIL missing bind source: {source}', flush=True)
                ok = False
    active = run(['docker', *compose, 'ps', '--status', 'running', '-q', service],
                 capture_output=True).stdout.strip()
    command = ['docker', *compose]
    command += ['exec', '-T', service] if active else ['run', '--rm', '--no-deps', service]
    command += ['/usr/local/bin/solo-entrypoint', 'python3', '/workspace/scripts/doctor.py', '--container']
    if require_renderer:
        command.append('--require-renderer')
    result = subprocess.run(command, check=False)
    return ok and result.returncode == 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--container', action='store_true')
    parser.add_argument('--require-renderer', action='store_true')
    parser.add_argument('--service', choices=['sim', 'companion'], default='sim')
    parser.add_argument('--compose', nargs=argparse.REMAINDER, default=['compose', '-f', 'compose.yaml'])
    args = parser.parse_args()
    try:
        ok = container_checks(args.require_renderer) if args.container else host_checks(args.compose, args.service, args.require_renderer)
    except (OSError, subprocess.CalledProcessError, KeyError, ValueError) as error:
        print(f'FAIL doctor: {error}', file=sys.stderr)
        ok = False
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
