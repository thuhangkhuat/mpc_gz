#!/usr/bin/env python3
"""Keep Docker attached while forwarding terminal cancellation through the mount."""

import argparse
from pathlib import Path
import signal
import subprocess


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cancel-file', type=Path, required=True)
    parser.add_argument('command', nargs=argparse.REMAINDER)
    args = parser.parse_args(argv)
    command = args.command[1:] if args.command[:1] == ['--'] else args.command
    if not command:
        parser.error('mission command required')
    # Register before spawning so cancellation during imports/preflight is kept.
    def cancel(signum, frame):
        args.cancel_file.parent.mkdir(parents=True, exist_ok=True)
        args.cancel_file.touch()
        print('Stopping mission; waiting for flight cleanup and landing.', flush=True)

    previous = {sig: signal.signal(sig, cancel)
                for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)}
    try:
        # A terminal Ctrl+C must not kill the Docker client before cleanup ends.
        process = subprocess.Popen(command, start_new_session=True)
        result = process.wait()
        return 130 if args.cancel_file.exists() else result
    finally:
        for sig, handler in previous.items():
            signal.signal(sig, handler)


if __name__ == '__main__':
    raise SystemExit(main())
