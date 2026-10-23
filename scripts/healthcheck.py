#!/usr/bin/env python3
"""Healthcheck needs no DDS participant; monitor refreshes this file continuously."""
import json
from pathlib import Path
import time

try:
    data = json.loads(Path('/workspace/runs/latest/health.json').read_text())
    healthy = data['ready'] and time.time() - data['updated_wall_unix'] < 10
except (OSError, ValueError, KeyError):
    healthy = False
raise SystemExit(0 if healthy else 1)
