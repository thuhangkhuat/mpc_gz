"""Construct every ROS adapter before attempting a Gazebo mission."""

from pathlib import Path
from types import SimpleNamespace

import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("casadi")
pytest.importorskip("gz.transport13")
from solo_mpc.controller_node import Controller
from solo_mpc.frame_node import Frames
from solo_mpc.perception_node import MarkerDetector
from solo_mpc.setpoint_node import Setpoints
from solo_mpc.settings import load_settings
from solo_mpc.target_node import MovingTarget


@pytest.mark.parametrize(
    "node_type", [Frames, MarkerDetector, MovingTarget, Controller, Setpoints]
)
def test_nodes_construct_without_conflicting_with_rclpy_apis(node_type, tmp_path):
    root = Path(__file__).resolve().parents[1] / "ros_ws/src/solo_mpc"
    scenario, settings, cfg = load_settings(
        root / "scenarios/small50_3uav.yaml", root / "config/controller.yaml"
    )
    args = SimpleNamespace(
        names=["test_uav"],
        settings_data=settings,
        cfg=cfg,
        scenario_data=scenario,
        run_dir=str(tmp_path),
        world="test_world",
    )
    rclpy.init()
    node = None
    try:
        node = node_type(args)
        assert node.get_name().startswith("mpc_")
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()


def node_args(tmp_path):
    root = Path(__file__).resolve().parents[1] / "ros_ws/src/solo_mpc"
    scenario, settings, cfg = load_settings(
        root / "scenarios/small50_3uav.yaml", root / "config/controller.yaml"
    )
    return SimpleNamespace(
        names=["test_uav"],
        settings_data=settings,
        cfg=cfg,
        scenario_data=scenario,
        run_dir=str(tmp_path),
        world="test_world",
    )


@pytest.mark.parametrize("accepted", [True, False])
def test_stop_plan_reference_keeps_nominal_failure_visible(tmp_path, monkeypatch, accepted):
    import json
    import numpy as np
    from geometry_msgs.msg import TwistStamped
    from mavros_msgs.msg import State
    from sensor_msgs.msg import LaserScan
    from solo_mpc.ros_utils import pose_message, seconds

    args = node_args(tmp_path)
    args.settings_data["controller_execution"] = "sequential"
    rclpy.init()
    node = Controller(args)
    try:
        name = args.names[0]
        stamp = node.get_clock().now().to_msg()
        node.pose(name, pose_message([8., 4., 3.], stamp))
        velocity = TwistStamped()
        velocity.header.stamp = stamp
        velocity.header.frame_id = "world"
        node.velocity(name, velocity)
        node.state(name, State(connected=True, armed=True, mode="GUIDED"))
        scan = LaserScan(angle_min=-np.pi, angle_increment=2*np.pi/360,
                         range_min=.1, range_max=3., ranges=[float("inf")] * 360)
        scan.header.stamp = stamp
        node.scan(name, scan)
        node.team.track.observe([9., 4.], seconds(stamp))
        references, status = [], []
        node.outputs[name] = (SimpleNamespace(publish=references.append),
                              SimpleNamespace(publish=lambda msg: None))
        node.status_pub = SimpleNamespace(publish=lambda msg: status.append(json.loads(msg.data)))
        result = dict(ok=accepted, nominal_ok=False, reason="stop_plan: mpc_failed",
                      position=np.array([8.1, 4.]), velocity=np.zeros(2),
                      acceleration=np.zeros(2), prediction=np.tile([8., 4., 0., 0., 0., 0.], (11, 1)))
        monkeypatch.setattr(node.team, "step", lambda *args, **kwargs: {name: result})
        node.tick()
        node.cycles.future.result(timeout=2)
        node.tick()
        assert len(references) == int(accepted)
        if accepted:
            assert references[0].position.z == args.settings_data["flight_altitude"]
        assert status[-1]["reason"] == "partial_solver_failure"
        assert status[-1]["vehicles"][name]["nominal_ok"] is False
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_per_vehicle_camera_subscribes_only_to_its_own_topics(tmp_path):
    args = node_args(tmp_path)
    args.names = ["alpha", "bravo", "charlie"]
    args.vehicle = "bravo"
    rclpy.init()
    node = MarkerDetector(args)
    try:
        assert set(node.history) == {"bravo"}
        assert set(node.publishers_keep) == {"bravo"}
        assert {s.topic_name for s in node.subscriptions_keep} == {
            "/bravo/mpc/pose", "/bravo/camera/image_raw", "/bravo/camera/camera_info",
        }
    finally:
        node.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize('fail', [False, True])
def test_run_node_records_constructor_success_or_failure(tmp_path, monkeypatch, fail):
    import json
    from solo_mpc import ros_utils

    args = node_args(tmp_path)
    args.startup_component = 'setpoints'
    monkeypatch.setattr(ros_utils, 'arguments', lambda _: (args, []))
    path = tmp_path / 'startup-components/setpoints.json'

    def construct(options):
        assert json.loads(path.read_text())['state'] == 'initializing'
        if fail:
            raise RuntimeError('constructor failed')
        return Setpoints(options)

    def spin(node):
        assert json.loads(path.read_text())['state'] == 'initialized'
        raise KeyboardInterrupt

    monkeypatch.setattr(rclpy, 'spin', spin)
    if fail:
        with pytest.raises(RuntimeError, match='constructor failed'):
            ros_utils.run_node(construct, 'Startup test')
        assert json.loads(path.read_text())['state'] == 'failed'
        assert 'constructor failed' in json.loads(path.read_text())['detail']
    else:
        ros_utils.run_node(construct, 'Startup test')
    assert not rclpy.ok()


def test_compass_error_does_not_rotate_enu_position_axes(tmp_path):
    import numpy as np
    from mavros_msgs.msg import State
    from scipy.spatial.transform import Rotation
    from solo_mpc.ros_utils import pose_message

    rclpy.init()
    node = Frames(node_args(tmp_path))
    try:
        node.state("test_uav", State(connected=True, armed=False))
        stamp = node.get_clock().now().to_msg()
        truth = pose_message(
            [8.0, 2.0, 0.195],
            stamp,
            Rotation.from_euler("z", 90, degrees=True).as_quat(),
        )
        # A ten-degree compass bias must not rotate a twenty-metre displacement.
        local = pose_message(
            [0.0, 0.0, 0.0], stamp, Rotation.from_euler("z", 80, degrees=True).as_quat()
        )
        node.ground_pose("test_uav", truth)
        for _ in range(10):
            node.local_pose("test_uav", local)
        rot, offset = node.transforms["test_uav"]
        assert np.allclose(rot.apply([20.0, 0.0, 3.0]) + offset, [28.0, 2.0, 3.195])
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_raw_setpoints_transform_enu_and_expire_to_zero_velocity(tmp_path):
    import time

    import numpy as np
    from mavros_msgs.msg import PositionTarget, State
    from rclpy.time import Time
    from solo_mpc.ros_utils import pose_message

    rclpy.init()
    node = Setpoints(node_args(tmp_path))
    try:
        name = "test_uav"
        sent = []
        node.publishers_keep[name] = SimpleNamespace(publish=sent.append)
        node.align[name] = {"yaw": np.pi / 2, "translation": [2.0, 3.0, 0.2]}
        node.enabled = True
        node.state(name, State(connected=True, armed=True, mode="GUIDED"))
        stamp = node.get_clock().now().to_msg()
        node.pose(name, pose_message([10.0, 5.0, 3.0], stamp))
        reference = PositionTarget()
        reference.header.frame_id = "world"
        reference.header.stamp = stamp
        reference.position.x, reference.position.y, reference.position.z = (
            10.0,
            5.0,
            3.0,
        )
        reference.velocity.x, reference.velocity.y = 0.4, 0.2
        reference.acceleration_or_force.x, reference.acceleration_or_force.y = 0.2, -0.1
        node.reference(name, reference)
        node.tick()
        command = sent[-1]
        assert np.allclose(
            [command.position.x, command.position.y, command.position.z],
            [2.0, -8.0, 2.8],
        )
        assert np.allclose([command.velocity.x, command.velocity.y], [0.2, -0.4])
        assert np.allclose(
            [command.acceleration_or_force.x, command.acceleration_or_force.y],
            [-0.1, -0.2],
        )
        assert (
            command.type_mask
            == PositionTarget.IGNORE_YAW | PositionTarget.IGNORE_YAW_RATE
        )
        # A slow simulator stretches low-rate FCU messages in wall time.
        state, _ = node.states[name]
        node.states[name] = (state, time.monotonic() - 5.0)
        node.tick()
        assert sent[-1].type_mask == command.type_mask
        node.state_stamps[name] -= 3.0
        node.tick()
        assert sent[-1].type_mask & PositionTarget.IGNORE_PX
        node.state(name, state)
        reference.header.stamp = Time(
            nanoseconds=node.get_clock().now().nanoseconds - 1_000_000_000
        ).to_msg()
        node.tick()
        expired = sent[-1]
        assert expired.type_mask & PositionTarget.IGNORE_PX
        assert np.allclose(
            [expired.velocity.x, expired.velocity.y, expired.velocity.z],
            [0.0, 0.0, 0.0],
        )
        pose, _ = node.poses[name]
        node.poses[name] = (pose, time.monotonic() - 5.0)
        node.tick()
        assert sent[-1].velocity.z == 0.0
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_callbacks_continue_with_frozen_cycle_and_obsolete_results_are_rejected(tmp_path, monkeypatch):
    import json
    import threading
    import numpy as np
    from geometry_msgs.msg import TwistStamped
    from mavros_msgs.msg import State
    from sensor_msgs.msg import LaserScan
    from rclpy.time import Time
    from solo_mpc.ros_utils import pose_message, seconds

    args = node_args(tmp_path)
    args.settings_data['controller_execution'] = 'sequential'
    rclpy.init()
    node = Controller(args)
    entered, release = threading.Event(), threading.Event()
    try:
        n = args.names[0]
        epoch = node.get_clock().now().nanoseconds
        current = [epoch]
        monkeypatch.setattr(node, 'get_clock', lambda: SimpleNamespace(now=lambda: Time(nanoseconds=current[0])))
        stamp = Time(nanoseconds=epoch).to_msg()
        node.pose(n, pose_message([8., 4., 3.], stamp))
        velocity = TwistStamped()
        velocity.header.stamp, velocity.header.frame_id = stamp, 'world'
        node.velocity(n, velocity)
        node.state(n, State(connected=True, armed=True, mode='GUIDED'))
        scan = LaserScan(angle_min=-np.pi, angle_increment=2*np.pi/360,
                         range_min=.1, range_max=3., ranges=[float('inf')] * 360)
        scan.header.stamp = stamp
        node.scan(n, scan)
        node.observation(n, pose_message([9., 4., 0.], stamp))
        references, statuses, calls = [], [], []
        node.outputs[n] = (SimpleNamespace(publish=references.append), SimpleNamespace(publish=lambda _: None))
        node.status_pub = SimpleNamespace(publish=lambda msg: statuses.append(json.loads(msg.data)))
        def optimize(states, scans, at, seers, tracker):
            calls.append(1)
            entered.set()
            assert release.wait(2)
            np.testing.assert_array_equal(states[n].position, [8.,4.,3.])
            np.testing.assert_array_equal(tracker.observation, [9.,4.])
            return {n: dict(ok=True, nominal_ok=True, position=np.array([8.,4.]),
                           velocity=np.zeros(2), acceleration=np.zeros(2),
                           prediction=np.tile([8.,4.,0.,0.,0.,0.], (11,1)), reason='track')}
        monkeypatch.setattr(node.team, 'step', optimize)
        node.tick()
        assert entered.wait(1)
        current[0] += 100_000_000
        later = Time(nanoseconds=current[0]).to_msg()
        node.pose(n, pose_message([8.2, 4., 3.], later))
        node.observation(n, pose_message([9.1, 4., 0.], later))
        node.tick()
        assert len(calls) == 1
        assert node.poses[n][0][0] == 8.2
        assert node.team.track.observed_at == seconds(later)
        release.set()
        node.cycles.future.result(timeout=2)
        current[0] += 500_000_000
        node.tick()
        assert not references
        assert statuses[-1]['reason'] == 'obsolete_cycle'
        assert statuses[-1]['target_input_fresh'] is True
        assert statuses[-1]['target_observation_age'] == pytest.approx(.5)
    finally:
        release.set()
        node.destroy_node()
        rclpy.shutdown()


def test_publisher_samples_delayed_trajectory_without_extending_expiry(tmp_path, monkeypatch):
    import numpy as np
    from mavros_msgs.msg import PositionTarget, State
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from builtin_interfaces.msg import Duration
    from rclpy.time import Time
    from solo_mpc.ros_utils import pose_message
    rclpy.init()
    node = Setpoints(node_args(tmp_path))
    try:
        current = [10_150_000_000]
        monkeypatch.setattr(node, 'get_clock', lambda: SimpleNamespace(now=lambda: Time(nanoseconds=current[0])))
        n = 'test_uav'
        sent = []
        node.publishers_keep[n] = SimpleNamespace(publish=sent.append)
        node.align[n] = dict(yaw=0., translation=[0., 0., 0.])
        node.enabled = True
        node.state(n, State(connected=True, armed=True, mode='GUIDED'))
        node.pose(n, pose_message([8., 4., 3.], Time(nanoseconds=current[0]).to_msg()))
        ref = PositionTarget()
        ref.header.frame_id = 'world'
        ref.header.stamp = Time(nanoseconds=10_000_000_000).to_msg()
        ref.position.z = 3.
        node.reference(n, ref)
        trajectory = JointTrajectory(header=ref.header, joint_names=['x', 'y'])
        trajectory.points = [JointTrajectoryPoint(
            positions=[8. + k * .1, 4.], velocities=[1., 0.], accelerations=[0., 0.],
            time_from_start=Duration(nanosec=k * 100_000_000)) for k in range(6)]
        node.trajectory(n, trajectory)
        node.tick()
        assert sent[-1].position.x == pytest.approx(8.15)
        assert sent[-1].velocity.x == 1.
        current[0] = 10_450_000_000
        node.tick()
        assert sent[-1].type_mask & PositionTarget.IGNORE_PX
        assert sent[-1].velocity.x == 0.
        current[0] = 9_990_000_000
        node.tick()
        assert sent[-1].type_mask & PositionTarget.IGNORE_PX
    finally:
        node.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize('mode', ['camera', 'ground_truth'])
def test_invalid_target_messages_do_not_renew_visibility(tmp_path, mode):
    import numpy as np
    from rclpy.time import Time
    from solo_mpc.ros_utils import pose_message
    args = node_args(tmp_path)
    args.settings_data['target_mode'] = mode
    args.settings_data['controller_execution'] = 'sequential'
    rclpy.init()
    node = Controller(args)
    try:
        now = node.get_clock().now()
        for position, frame, stamp in [([1,2,0], 'local', now.to_msg()),
                                      ([np.nan,2,0], 'world', now.to_msg()),
                                      ([1,2,0], 'world', Time(nanoseconds=now.nanoseconds-2_000_000_000).to_msg()),
                                      ([1,2,0], 'world', Time(nanoseconds=now.nanoseconds+1_000_000_000).to_msg())]:
            node.observation('ground_truth' if mode == 'ground_truth' else args.names[0],
                             pose_message(position, stamp, frame=frame))
        assert not node.observed and node.team.track.observed_at is None
    finally:
        node.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize('mode', ['camera', 'ground_truth'])
def test_controller_accepts_only_the_selected_target_source(tmp_path, monkeypatch, mode):
    import numpy as np
    from rclpy.time import Time
    from solo_mpc.ros_utils import pose_message

    args = node_args(tmp_path)
    args.settings_data['target_mode'] = mode
    rclpy.init()
    node = Controller(args)
    try:
        topics = {s.topic_name for s in node.subscriptions_keep}
        assert ('/target/ground_truth' in topics) == (mode == 'ground_truth')
        assert ('/test_uav/target/observation' in topics) == (mode == 'camera')
        current = Time(nanoseconds=10_000_000_000)
        monkeypatch.setattr(node, 'get_clock', lambda: SimpleNamespace(now=lambda: current))
        source = 'ground_truth' if mode == 'ground_truth' else args.names[0]
        other = args.names[0] if mode == 'ground_truth' else 'ground_truth'
        node.observation(other, pose_message([99, 99, 0], current.to_msg()))
        assert node.team.track.x is None
        for at, point in [(9.7, [1, 1, 0]), (9.8, [2, 1, 0]), (9.9, [2, 2, 0])]:
            stamp = Time(nanoseconds=int(at * 1e9)).to_msg()
            node.observation(source, pose_message(point, stamp))
            if mode == 'ground_truth':
                np.testing.assert_array_equal(node.team.track.x[:2], point[:2])
        np.testing.assert_array_equal(node.team.track.observation, [2, 2])
        assert node.team.track.observed_at == pytest.approx(9.9)
    finally:
        node.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize('at_boundary', [False, True])
def test_external_target_cycles_use_upstream_visibility_and_expire(tmp_path, monkeypatch, at_boundary):
    import json
    import numpy as np
    from geometry_msgs.msg import TwistStamped
    from mavros_msgs.msg import State
    from rclpy.time import Time
    from sensor_msgs.msg import LaserScan
    from solo_mpc.ros_utils import pose_message

    args = node_args(tmp_path)
    args.settings_data['target_mode'] = 'ground_truth'
    rclpy.init()
    node = Controller(args)
    try:
        current = [10_000_000_000]
        monkeypatch.setattr(node, 'get_clock', lambda: SimpleNamespace(
            now=lambda: Time(nanoseconds=current[0])))
        stamp = Time(nanoseconds=current[0]).to_msg()
        name = args.names[0]
        node.pose(name, pose_message([8, 4, 3], stamp))
        velocity = TwistStamped()
        velocity.header.stamp, velocity.header.frame_id = stamp, 'world'
        node.velocity(name, velocity)
        node.state(name, State(connected=True, armed=True, mode='GUIDED'))
        scan = LaserScan(angle_min=-np.pi, angle_increment=2*np.pi/360,
                         range_min=.1, range_max=3., ranges=[float('inf')] * 360)
        scan.header.stamp = stamp
        node.scan(name, scan)
        target = [8 + args.cfg['VIEWING_RADIUS'] if at_boundary else 20, 4, 0]
        node.observation('ground_truth', pose_message(target, stamp))
        statuses, calls = [], []
        node.status_pub = SimpleNamespace(publish=lambda msg: statuses.append(json.loads(msg.data)))

        def step(states, scans, at, seers, tracker):
            calls.append(at)
            assert seers == ({name} if at_boundary else set())
            np.testing.assert_array_equal(tracker.position(at), target[:2])
            return {}

        monkeypatch.setattr(node.team, 'step', step)
        node.tick()
        node.cycles.future.result(timeout=2)
        node.tick()
        assert calls == [10.]
        assert statuses[-1]['reason'] == 'ready'
        assert statuses[-1]['target_input_fresh'] is True
        # A feed outage cannot use the camera mode's longer SEARCH/coast budget.
        current[0] += int((args.settings_data['target_timeout_sim'] + .1) * 1e9)
        node.tick()
        assert statuses[-1]['reason'] == 'target_unavailable'
        assert statuses[-1]['target_input_fresh'] is False
        assert node.cycles.future is None
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_target_timestamps_match_route_and_pause_resume_reset(tmp_path, monkeypatch):
    import numpy as np
    from rclpy.time import Time
    from std_srvs.srv import SetBool, Trigger
    from solo_mpc.ros_utils import pose_arrays, seconds

    args = node_args(tmp_path)
    args.scenario_data = {'tar_max_speed': 2, 'target': {
        'waypoints': [[0, 0], [10, 0], [10, 5]], 'speeds': [2, 5],
    }}
    args.cfg['TAR_SMOOTH_ENABLE'] = False
    rclpy.init()
    node = MovingTarget(args)
    try:
        current = [10_000_000_000]
        monkeypatch.setattr(node, 'get_clock', lambda: SimpleNamespace(
            now=lambda: Time(nanoseconds=current[0])))
        published = []
        node.publisher = SimpleNamespace(publish=published.append)
        node.gz = SimpleNamespace(request=lambda *a: (True, SimpleNamespace(data=True)))
        node.tick()
        node.set_running(SetBool.Request(data=True), SetBool.Response())
        current[0] = 12_500_000_000
        node.tick()
        np.testing.assert_array_equal(pose_arrays(published[-1])[0][:2], [5, 0])
        node.set_running(SetBool.Request(data=False), SetBool.Response())
        current[0] = 20_000_000_000
        node.tick()
        np.testing.assert_array_equal(pose_arrays(published[-1])[0][:2], [5, 0])
        node.set_running(SetBool.Request(data=True), SetBool.Response())
        current[0] = 23_000_000_000
        node.tick()
        np.testing.assert_array_equal(pose_arrays(published[-1])[0][:2], [10, 2.5])
        current[0] = 30_000_000_000
        node.tick()
        np.testing.assert_array_equal(pose_arrays(published[-1])[0][:2], [10, 5])
        assert not node.running
        node.reset(Trigger.Request(), Trigger.Response())

        def delayed_request(*a):
            current[0] += 250_000_000
            return True, SimpleNamespace(data=True)

        node.gz.request = delayed_request
        node.tick()
        np.testing.assert_array_equal(pose_arrays(published[-1])[0][:2], [0, 0])
        assert seconds(published[-1].header.stamp) == 30.
        assert published[-1].header.frame_id == 'world'
        # An unsuccessful Gazebo pose update must not masquerade as fresh truth.
        node.gz.request = lambda *a: (False, None)
        count = len(published)
        node.tick()
        assert len(published) == count
    finally:
        node.destroy_node()
        rclpy.shutdown()
