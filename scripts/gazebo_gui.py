"""Open the Gazebo GUI client with a free camera; the server keeps running on exit."""

import subprocess


def main():
    gui = subprocess.Popen(["gz", "sim", "-g", "-v", "3"])
    try:
        return gui.wait()
    except KeyboardInterrupt:
        # The GUI shares the foreground process group and also receives Ctrl+C.
        return gui.wait()
    finally:
        if gui.poll() is None:
            gui.terminate()
            gui.wait()


if __name__ == "__main__":
    raise SystemExit(main())
