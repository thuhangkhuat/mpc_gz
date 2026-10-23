"""Catch conflicts with rclpy Node APIs before starting a mission."""
import pytest

rclpy = pytest.importorskip('rclpy')

from solo_bringup.flight import Flight


def test_flight_node_can_create_command_clients():
    rclpy.init()
    node = None
    try:
        node = Flight([{'name': 'construction_test'}], timeout=1)
        assert len(tuple(node.clients)) == 3
        assert {subscription.topic_name for subscription in node.subscriptions} == {
            '/clock', '/construction_test/mavros/state',
            '/construction_test/mavros/local_position/pose',
        }
        assert node.get_parameter('use_sim_time').value is True
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
