"""Send camera images over ROS and verify each UAV's finalized video."""

import json
import time

import cv2
import numpy as np
import pytest

rclpy = pytest.importorskip("rclpy")
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State

from solo_mpc.camera_recorder import CameraRecorder


@pytest.mark.parametrize("overhead", [False, True])
def test_ros_camera_feeds_are_recorded_separately_and_closed_on_shutdown(tmp_path, overhead):
    names = ["overhead"] if overhead else [
        "camera_test_alpha", "camera_test_bravo", "camera_test_charlie",
    ]
    topics = ["/overhead/camera/image_raw"] if overhead else [
        f"/{name}/camera/image_raw" for name in names
    ]
    colors = [(0, 0, 255)] if overhead else [(0, 0, 255), (0, 255, 0), (255, 0, 255)]
    width, height, fps = (1920, 1080, 20.0) if overhead else (160, 160, 8.0)
    rclpy.init()
    source = rclpy.create_node("camera_test_source")
    vehicles = ['camera_test_alpha', 'camera_test_bravo', 'camera_test_charlie']
    recorder = CameraRecorder(names, tmp_path, fps, overhead=overhead, vehicle_names=vehicles)
    try:
        if overhead:
            assert not recorder.image_subscriptions
        else:
            assert set(recorder.image_subscriptions) == set(names)
        for vehicle in vehicles:
            recorder.state(vehicle, State(connected=True, armed=False))
            pose = PoseStamped()
            pose.pose.position.z = 10.0
            recorder.pose(vehicle, pose)
            recorder.state(vehicle, State(connected=True, armed=True))
            pose.pose.position.z = 10.25
            recorder.pose(vehicle, pose)
        assert {s.topic_name for s in recorder.subscriptions_keep if s.msg_type is Image} == set(topics)
        publishers = [source.create_publisher(
            Image, topic, qos_profile_sensor_data,
        ) for topic in topics]
        deadline = time.monotonic() + 10
        while any(pub.get_subscription_count() == 0 for pub in publishers):
            assert time.monotonic() < deadline, "Camera subscribers did not connect"
            rclpy.spin_once(recorder, timeout_sec=0.01)
        bridge = CvBridge()
        for i in range(3):
            for pub, color in zip(publishers, colors):
                msg = bridge.cv2_to_imgmsg(np.full((height, width, 3), color, np.uint8), "bgr8")
                msg.header.stamp.sec = 12
                msg.header.stamp.nanosec = i * int(1_000_000_000 / fps)
                pub.publish(msg)
            while any(video.frames < i + 1 for video in recorder.videos.values()):
                assert time.monotonic() < deadline, "Camera frames did not arrive"
                rclpy.spin_once(recorder, timeout_sec=0.01)
    finally:
        recorder.destroy_node()
        source.destroy_node()
        rclpy.shutdown()
    for name, color in zip(names, colors):
        capture = cv2.VideoCapture(str(tmp_path / f"{name}.avi"))
        try:
            assert capture.get(cv2.CAP_PROP_FPS) == fps
            for _ in range(3):
                ok, frame = capture.read()
                assert ok
                assert frame.shape[:2] == (height, width)
                assert np.allclose(frame[80, 80], color, atol=5)
            assert not capture.read()[0]
        finally:
            capture.release()
        timestamps = [json.loads(row) for row in (
            tmp_path / f"{name}.timestamps.jsonl"
        ).read_text().splitlines()]
        assert [row["stamp_ns"] for row in timestamps] == [
            12_000_000_000 + i * int(1_000_000_000 / fps) for i in range(3)
        ]


@pytest.mark.parametrize('overhead', [False, True])
def test_recording_waits_for_armed_takeoff_and_uses_each_vehicles_ground_level(tmp_path, overhead):
    vehicles = ['gate_alpha', 'gate_bravo']
    names = ['overhead'] if overhead else vehicles
    rclpy.init()
    recorder = CameraRecorder(names, tmp_path, 8, overhead=overhead, vehicle_names=vehicles)
    frame = CvBridge().cv2_to_imgmsg(np.zeros((32, 32, 3), np.uint8), 'bgr8')

    def images():
        for name in names:
            recorder.image(name, frame)

    def pose(name, z):
        msg = PoseStamped()
        msg.pose.position.z = z
        recorder.pose(name, msg)

    try:
        images()
        for name in vehicles:
            recorder.state(name, State(connected=True, armed=False))
            pose(name, 10.0)
        if overhead:
            assert not recorder.image_subscriptions
        images()
        for name in vehicles:
            recorder.state(name, State(connected=True, armed=True))
        if overhead:
            assert len(recorder.image_subscriptions) == 1
        images()  # Armed on the ground is still idle.
        assert all(v.frames == 0 for v in recorder.videos.values())
        assert not list(tmp_path.glob('*.avi'))
        pose(vehicles[1], 10.3)  # Only bravo is flying.
        images()
        if overhead:
            assert recorder.videos['overhead'].frames == 1
        else:
            assert recorder.videos[vehicles[0]].frames == 0
            assert recorder.videos[vehicles[1]].frames == 1
        pose(vehicles[0], 10.3)
        images()
        assert all(v.frames > 0 for v in recorder.videos.values())
    finally:
        recorder.destroy_node()
        rclpy.shutdown()
