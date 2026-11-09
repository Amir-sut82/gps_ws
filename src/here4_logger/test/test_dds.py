"""Opt-in DDS integration test; publishes only within an isolated test domain."""

import csv
import os
import signal
import subprocess
import sys
import time

import pytest


@pytest.mark.skipif(os.environ.get('HERE4_RUN_DDS_TESTS') != '1',
                    reason='Set HERE4_RUN_DDS_TESTS=1 in a separate ROS domain')
def test_dds_sessions_and_sigint(tmp_path):
    import rclpy
    from px4_msgs.msg import SensorGps, VehicleStatus
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

    from here4_logger.node import px4_topic

    assert px4_topic('vehicle_gps_position', SensorGps) == '/fmu/out/vehicle_gps_position'
    assert px4_topic('vehicle_status', VehicleStatus) == '/fmu/out/vehicle_status_v1'
    assert px4_topic('vehicle_status', VehicleStatus, '/px4_1/') == (
        '/px4_1/fmu/out/vehicle_status_v1')
    assert os.environ.get('ROS_DOMAIN_ID') == '88', 'Use ROS_DOMAIN_ID=88 for this test'
    rclpy.init()
    source = rclpy.create_node('here4_test_source')
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    gps_pub = source.create_publisher(
        SensorGps, '/fmu/out/vehicle_gps_position', qos)
    status_pub = source.create_publisher(
        VehicleStatus, '/fmu/out/vehicle_status_v1', qos)
    output = tmp_path / 'csv'
    stdout = (tmp_path / 'logger.txt').open('w')
    env = dict(os.environ, ROS_LOG_DIR=str(tmp_path / 'ros_log'))
    child = subprocess.Popen([
        sys.executable, '-m', 'here4_logger.node', '--ros-args',
        '-p', f'output_directory:={output}', '-p', 'print_rate_hz:=10.0',
    ], stdout=stdout, stderr=subprocess.STDOUT, env=env)

    def wait_for(predicate, timeout=10.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(source, timeout_sec=0.03)
            if predicate():
                return
            assert child.poll() is None, (tmp_path / 'logger.txt').read_text()
        pytest.fail((tmp_path / 'logger.txt').read_text())

    def files():
        return sorted(output.glob('*.csv'))

    def rows(path):
        with path.open() as stream:
            return list(csv.DictReader(stream))

    timestamp = 1000

    def send_status(armed, arm_time):
        nonlocal timestamp
        timestamp += 1000
        message = VehicleStatus()
        message.timestamp = timestamp
        message.arming_state = 2 if armed else 1
        message.armed_time = arm_time
        status_pub.publish(message)

    def send_gps():
        nonlocal timestamp
        timestamp += 1000
        message = SensorGps()
        message.timestamp = timestamp
        message.latitude_deg = 35.12345678912345
        message.longitude_deg = 51.12345678912345
        message.altitude_msl_m = 1250.123456789
        message.fix_type = 6
        message.satellites_used = 24
        gps_pub.publish(message)

    try:
        wait_for(lambda: gps_pub.get_subscription_count() == 1
                 and status_pub.get_subscription_count() == 1)
        wait_for(lambda: ('here4_logger', '/') in source.get_node_names_and_namespaces())
        topics = source.get_publisher_names_and_types_by_node('here4_logger', '/')
        assert not any(name.startswith('/fmu/in/') for name, _ in topics)
        send_status(False, 0)
        send_gps()
        wait_for(lambda: 'DISARMED' in (tmp_path / 'logger.txt').read_text())
        assert files() == []

        send_status(True, 10_000)
        wait_for(lambda: len(files()) == 1)
        first = files()[0]
        send_gps()
        wait_for(lambda: len(rows(first)) == 1)
        send_status(True, 10_000)
        send_gps()
        wait_for(lambda: len(rows(first)) == 2)
        assert len(files()) == 1
        send_status(False, 10_000)
        wait_for(lambda: 'disarmed' in (tmp_path / 'logger.txt').read_text())

        send_status(True, 20_000)
        wait_for(lambda: len(files()) == 2)
        second = next(path for path in files() if path != first)
        send_gps()
        wait_for(lambda: len(rows(second)) == 1)
        child.send_signal(signal.SIGINT)
        assert child.wait(timeout=10.0) == 0
        text = (tmp_path / 'logger.txt').read_text()
        assert 'logger shutdown' in text
        assert 'Failed to publish' not in text
        assert 'RTK_FIXED' in text
        assert float(rows(first)[0]['latitude_deg']) == 35.12345678912345
        assert len(rows(first)) == 2 and len(rows(second)) == 1
    finally:
        if child.poll() is None:
            child.terminate()
            child.wait(timeout=10.0)
        stdout.close()
        source.destroy_node()
        rclpy.shutdown()
