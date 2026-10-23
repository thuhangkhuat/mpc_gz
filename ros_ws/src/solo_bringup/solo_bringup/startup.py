"""Startup observations shared by launch, ROS nodes and the host wait helper."""

import json
import os
from pathlib import Path
import time


def write_status(path, payload):
    path = Path(path)
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(payload, indent=2) + '\n')
    temporary.replace(path)


def startup_blockers(run, startup):
    """Require live processes and constructor completion from this launch."""
    if not startup or not startup.get('processes'):
        return ['waiting for launch process inventory']
    blockers = []
    for name, item in startup['processes'].items():
        if item['state'] == 'exited':
            raise RuntimeError(f"{name} exited during startup (code {item['exit_code']})")
        if item['state'] != 'running':
            blockers.append(f'{name}: not started')
            continue
        if not item.get('initializes'):
            continue
        try:
            component = json.loads((Path(run) / 'startup-components' / f'{name}.json').read_text())
        except (OSError, ValueError):
            component = {}
        if (component.get('pid') != item.get('pid') or
                component.get('updated_wall_unix', 0) < startup['started_wall_unix']):
            component = {}
        if component.get('state') == 'failed':
            raise RuntimeError(f"{name} failed during startup: {component['detail']}")
        if component.get('state') != 'initialized':
            blockers.append(f'{name}: initializing')
    return blockers


class StartupProgress:
    """The launch process owns the process inventory and lifecycle events."""

    def __init__(self, run):
        self.path = Path(run) / 'startup.json'
        self.started = time.time()
        self.phase = 'Preparing launch'
        self.processes = {}
        self.save()

    def save(self):
        write_status(self.path, {
            'started_wall_unix': self.started, 'updated_wall_unix': time.time(),
            'phase': self.phase, 'processes': self.processes,
        })

    def stage(self, phase):
        self.phase = phase
        self.save()

    def register(self, name, initializes=False):
        self.processes[name] = {'state': 'pending', 'initializes': initializes}
        self.save()

    def running(self, name, pid):
        self.processes[name].update(state='running', pid=pid, started_wall_unix=time.time())
        if all(item['state'] != 'pending' for item in self.processes.values()):
            self.phase = 'Processes launched'
        self.save()

    def exited(self, name, code):
        self.processes[name].update(state='exited', exit_code=code)
        self.save()


class ComponentProgress:
    """Each MPC process owns its constructor status; no concurrent JSON writers."""

    def __init__(self, run, name):
        self.path = Path(run) / 'startup-components' / f'{name}.json'
        self.path.parent.mkdir(exist_ok=True)

    def update(self, state, detail):
        write_status(self.path, {
            'state': state, 'detail': detail, 'pid': os.getpid(),
            'updated_wall_unix': time.time(),
        })
