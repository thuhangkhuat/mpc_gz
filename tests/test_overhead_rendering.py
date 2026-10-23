"""Render an isolated Gazebo scene, move its target, and decode the saved video."""

import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid
import xml.etree.ElementTree as ET

import cv2
import numpy as np
import pytest
from solo_bringup.configuration import load_config
from solo_mpc.settings import load_settings
from solo_mpc.world import bridge_config, generate_mpc_world

ROOT = Path(__file__).resolve().parents[1]
PKG = ROOT / "ros_ws/src/solo_mpc"


@pytest.mark.skipif(not Path("/opt/ardupilot_gazebo").exists(),
                    reason="Requires Gazebo and the pinned models in the simulation image")
def test_overhead_video_renders_full_hd_and_follows_target(tmp_path):
    fleet = load_config(ROOT / "config/fleet.yaml")
    scenario, settings, _ = load_settings(PKG / 'scenarios/small50_3uav.yaml',
                                         PKG / 'config/controller.yaml')
    # Move to a clear route waypoint. The old (12, 4) fixture was inside a box,
    # so the obstacle legitimately hid the target in the final camera frames.
    destination = scenario['target']['waypoints'][1]
    world, _ = generate_mpc_world(
        fleet, "/opt/ardupilot_gazebo", tmp_path / "world.sdf",
        PKG / "scenarios/small50_3uav.yaml", PKG / "config/controller.yaml",
        record_overhead=True,
    )
    tree = ET.parse(world)
    # Keep real UAV visuals but disable flight/sensor systems: this test neither
    # binds FCU ports nor needs to start SITL, MAVROS or a flying mission.
    for uav in fleet["uavs"]:
        model = tree.getroot().find(f"world/model[@name='{uav['name']}']")
        static = model.find("static")
        if static is None:
            static = ET.SubElement(model, "static")
        static.text = "true"
        for parent in model.iter():
            for child in list(parent):
                if child.tag in ("plugin", "sensor"):
                    parent.remove(child)
    tree.write(world)
    config = bridge_config([], fleet["world_name"], tmp_path / "bridge.yaml",
                           record_overhead=True)
    env = {**os.environ, "GZ_PARTITION": f"overhead-test-{uuid.uuid4().hex}",
           'ROS_DOMAIN_ID': str(100 + uuid.uuid4().int % 100)}
    start_flight = tmp_path / 'start-flight'
    telemetry_ready = tmp_path / 'telemetry-ready'
    # Feed flight telemetry in the isolated ROS domain without running an FCU.
    telemetry = '''from pathlib import Path
import sys
import rclpy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from rclpy.qos import qos_profile_sensor_data
rclpy.init()
node = rclpy.create_node('overhead_test_telemetry')
state = node.create_publisher(State, '/uav1/mavros/state', qos_profile_sensor_data)
pose = node.create_publisher(PoseStamped, '/uav1/mavros/local_position/pose', qos_profile_sensor_data)
ground_ticks = 0
while rclpy.ok():
    flying = Path(sys.argv[1]).exists()
    state.publish(State(connected=True, armed=flying))
    message = PoseStamped()
    message.pose.position.z = 10.3 if flying else 10.0
    pose.publish(message)
    if not flying and state.get_subscription_count() and pose.get_subscription_count():
        ground_ticks += 1
        if ground_ticks >= 10:
            Path(sys.argv[2]).touch()
    rclpy.spin_once(node, timeout_sec=0.05)
'''
    commands = [
        ["gz", "sim", "-s", "-r", "--headless-rendering", str(world)],
        ["ros2", "run", "ros_gz_bridge", "parameter_bridge", "--ros-args",
         "-p", f"config_file:={config}"],
        [sys.executable, '-c', telemetry, str(start_flight), str(telemetry_ready)],
        [sys.executable, "-m", "solo_mpc.camera_recorder", "--overhead",
         '--fleet', str(ROOT / 'config/fleet.yaml'),
         "--output-dir", str(tmp_path), "--fps", "20"],
    ]
    processes = []
    logs = []
    video = tmp_path / "overhead.avi"
    deadline = time.monotonic() + 60

    def wait_for_size(size):
        while not video.exists() or video.stat().st_size < size:
            assert all(p.poll() is None for p in processes), [
                path.read_text() for path in logs
            ]
            assert time.monotonic() < deadline, [path.read_text() for path in logs]
            time.sleep(0.1)

    try:
        for i, command in enumerate(commands):
            path = tmp_path / f"process-{i}.log"
            logs.append(path)
            with path.open("w") as output:
                processes.append(subprocess.Popen(command, env=env, stdout=output,
                                                   stderr=subprocess.STDOUT))
        while (not telemetry_ready.exists() or
               'Waiting for armed UAV takeoff' not in logs[-1].read_text()):
            assert all(p.poll() is None for p in processes), [path.read_text() for path in logs]
            assert time.monotonic() < deadline, [path.read_text() for path in logs]
            time.sleep(0.1)
        # No extra image subscriber: the lazy bridge must connect when the
        # recorder receives the armed state, then produce a real Full HD video.
        assert not video.exists(), 'Disarmed startup must not record'
        while True:
            info = subprocess.run(
                ['gz', 'topic', '-i', '-t', '/overhead/camera/image_raw'],
                env=env, capture_output=True, text=True, timeout=10,
            )
            if 'gz.msgs.Image' in info.stdout:
                break
            assert time.monotonic() < deadline, info.stdout + info.stderr
            assert all(p.poll() is None for p in processes), [path.read_text() for path in logs]
        assert 'No subscribers on topic' in info.stdout, info.stdout
        start_flight.touch()
        wait_for_size(256_000)
        before = video.stat().st_size
        move = subprocess.run(
            ["gz", "service", "-s", f"/world/{fleet['world_name']}/set_pose",
             "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
             "--timeout", "5000", "--req",
             'name: "tracking_target", position: {x: %s, y: %s, z: %s}, orientation: {w: 1}'
             % (destination[0], destination[1], settings['target_height'])],
            env=env, capture_output=True, text=True, timeout=10,
        )
        assert move.returncode == 0 and "data: true" in move.stdout, move.stdout + move.stderr
        wait_for_size(before + 512_000)
    finally:
        # Stop the recorder first and let it write the AVI index before decoding.
        for process in reversed(processes):
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=5)

    capture = cv2.VideoCapture(str(video))
    frames = []
    try:
        assert capture.get(cv2.CAP_PROP_FPS) == 20
        while True:
            ok, frame = capture.read()
            if not ok:
                break
            assert frame.shape == (1080, 1920, 3)
            frames.append(frame)
    finally:
        capture.release()
    assert len(frames) >= 5
    for frame in (frames[0], frames[-1]):
        # The magenta marker remains centered after translating the target.
        b, g, r = cv2.split(frame)
        ys, xs = np.nonzero((b > 150) & (r > 150) & (g < 100))
        assert len(xs) >= 4
        assert abs(xs.mean() - 960) < 20
        assert abs(ys.mean() - 540) < 20
    assert np.mean(cv2.absdiff(frames[0], frames[-1])) > 1
    timestamps = [json.loads(row)["stamp_ns"] for row in
                  (tmp_path / "overhead.timestamps.jsonl").read_text().splitlines()]
    assert len(timestamps) == len(frames)
    assert all(b > a for a, b in zip(timestamps, timestamps[1:]))
