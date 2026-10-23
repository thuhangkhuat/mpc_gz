"""Terminal cancellation must reach the remote supervisor without killing its client."""

from pathlib import Path
import signal
import subprocess
import sys
import time

import pytest

ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize('sig', [signal.SIGINT, signal.SIGTERM, signal.SIGHUP])
def test_cancellation_keeps_client_alive_until_cleanup_finishes(tmp_path, sig):
    cancel = tmp_path / 'mission.cancel'
    ready = tmp_path / 'ready'
    cleaned = tmp_path / 'cleaned'
    worker = tmp_path / 'worker.py'
    worker.write_text('''from pathlib import Path
import sys, time
cancel, ready, cleaned = map(Path, sys.argv[1:])
ready.touch()
while not cancel.exists():
    time.sleep(0.01)
time.sleep(0.1)
cleaned.touch()
''')
    process = subprocess.Popen([
        sys.executable, str(ROOT / 'scripts/run_mission.py'), '--cancel-file', str(cancel),
        '--', sys.executable, str(worker), str(cancel), str(ready), str(cleaned),
    ], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    try:
        deadline = time.monotonic() + 5
        while not ready.exists():
            assert process.poll() is None
            assert time.monotonic() < deadline
            time.sleep(0.01)
        process.send_signal(sig)
        output, errors = process.communicate(timeout=5)
        assert process.returncode == 130, errors
        assert 'waiting for flight cleanup' in output
        assert cleaned.exists()
    finally:
        if process.poll() is None:
            cancel.touch()
            process.kill()
            process.wait()
