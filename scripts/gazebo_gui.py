"""Open Gazebo with a wide, elevated follow view for the MPC target."""

from pathlib import Path
import os
import subprocess
import time
import xml.etree.ElementTree as ET

import yaml

from solo_mpc.scene_view import FOLLOW_OFFSET, TARGET, follow_offset


def has_target(world_path):
    if not world_path.is_file():
        return False
    world = ET.parse(world_path).getroot().find("world")
    return world is not None and world.find(f"model[@name='{TARGET}']") is not None


def follow_target(gui, timeout=60.0, offset=FOLLOW_OFFSET):
    from gz.msgs10.cameratrack_pb2 import CameraTrack
    from gz.transport13 import Node

    node = Node()
    publisher = node.advertise("/gui/track", CameraTrack)
    confirmed = 0

    def status(message):
        nonlocal confirmed
        if message.track_mode == CameraTrack.FOLLOW and message.follow_target.name == TARGET:
            confirmed += 1
        else:
            confirmed = 0

    node.subscribe(CameraTrack, "/gui/currently_tracked", status)
    request = CameraTrack()
    request.track_mode = CameraTrack.FOLLOW
    request.follow_target.name = TARGET
    request.follow_offset.x, request.follow_offset.y, request.follow_offset.z = offset
    request.follow_pgain = 0.08
    request.track_pgain = 0.08
    deadline = time.monotonic() + timeout
    next_publish = 0.0
    while gui.poll() is None and time.monotonic() < deadline:
        # Require several status updates so a target still loading into the
        # render scene does not count as a successfully established follow.
        if confirmed >= 3:
            print(f"Gazebo view follows the pink target from {offset[2]:g} m above.", flush=True)
            return True
        if publisher.has_connections() and time.monotonic() >= next_publish:
            publisher.publish(request)
            next_publish = time.monotonic() + 1.0
        time.sleep(0.1)
    if gui.poll() is None:
        print("Gazebo camera follow did not become ready; reopen the GUI to retry.", flush=True)
    return False


def main():
    world_path = Path(os.environ.get("RUNS_ROOT", "/workspace/runs")) / "latest/world.sdf"
    track = has_target(world_path)
    gui = subprocess.Popen(["gz", "sim", "-g", "-v", "3"])
    try:
        if track:
            settings_path = world_path.parent / 'controller.yaml'
            settings = yaml.safe_load(settings_path.read_text()) if settings_path.is_file() else {}
            follow_target(gui, offset=follow_offset(settings))
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
