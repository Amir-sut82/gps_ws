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
@pytest.mark.parametrize('source_kind', ['gps', 'ekf'])
def test_dds_sessions_and_sigint(tmp_path, source_kind):
    import rclpy
    from px4_msgs.msg import SensorGps, VehicleGlobalPosition, VehicleStatus
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

    from here4_logger.node import px4_topic

    assert px4_topic('vehicle_gps_position', SensorGps) == '/fmu/out/vehicle_gps_position'
    assert px4_topic('vehicle_status', VehicleStatus) == '/fmu/out/vehicle_status_v1'
    assert px4_topic('vehicle_status', VehicleStatus, '/px4_1/') == (
        '/px4_1/fmu/out/vehicle_status_v1')
    assert px4_topic('vehicle_global_position', VehicleGlobalPosition) == (
        '/fmu/out/vehicle_global_position')
    assert os.environ.get('ROS_DOMAIN_ID') == '88', 'Use ROS_DOMAIN_ID=88 for this test'
    is_gps = source_kind == 'gps'
    message_type = SensorGps if is_gps else VehicleGlobalPosition
    position_topic = ('/fmu/out/vehicle_gps_position' if is_gps
                      else '/fmu/out/vehicle_global_position')
    node_name = 'here4_logger' if is_gps else 'ekf_position_logger'
    module = 'here4_logger.node' if is_gps else 'here4_logger.ekf_node'
    rclpy.init()
    source = rclpy.create_node('here4_test_source')
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    position_pub = source.create_publisher(message_type, position_topic, qos)
    status_pub = source.create_publisher(
        VehicleStatus, '/fmu/out/vehicle_status_v1', qos)
    output = tmp_path / 'csv'
    stdout = (tmp_path / 'logger.txt').open('w')
    env = dict(os.environ, ROS_LOG_DIR=str(tmp_path / 'ros_log'))
    child = subprocess.Popen([
        sys.executable, '-m', module, '--ros-args',
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

    def send_position(valid=True):
        nonlocal timestamp
        timestamp += 1000
        message = message_type()
        message.timestamp = timestamp
        if is_gps:
            message.latitude_deg = 35.12345678912345
            message.longitude_deg = 51.12345678912345
            message.altitude_msl_m = 1250.123456789
            message.fix_type = 6
            message.satellites_used = 24
        else:
            message.lat = 35.12345678912345
            message.lon = 51.12345678912345
            message.alt = 1250.125
            message.lat_lon_valid = valid
            message.alt_valid = valid
            message.dead_reckoning = not valid
            message.eph = 0.04
            message.epv = 0.08
        position_pub.publish(message)

    try:
        wait_for(lambda: position_pub.get_subscription_count() == 1
                 and status_pub.get_subscription_count() == 1)
        wait_for(lambda: (node_name, '/') in source.get_node_names_and_namespaces())
        topics = source.get_publisher_names_and_types_by_node(node_name, '/')
        assert not any(name.startswith('/fmu/in/') for name, _ in topics)
        send_status(False, 0)
        send_position()
        wait_for(lambda: 'DISARMED' in (tmp_path / 'logger.txt').read_text())
        assert files() == []

        send_status(True, 10_000)
        wait_for(lambda: len(files()) == 1)
        first = files()[0]
        assert first.name.startswith('position_' if is_gps else 'ekf_position_')
        send_position()
        wait_for(lambda: len(rows(first)) == 1)
        send_status(True, 10_000)
        send_position()
        wait_for(lambda: len(rows(first)) == 2)
        assert len(files()) == 1
        send_status(False, 10_000)
        wait_for(lambda: 'disarmed' in (tmp_path / 'logger.txt').read_text())

        send_status(True, 20_000)
        wait_for(lambda: len(files()) == 2)
        second = next(path for path in files() if path != first)
        send_position(valid=False)
        wait_for(lambda: len(rows(second)) == 1)
        if not is_gps:
            assert rows(second)[0]['position_3d_valid'] == 'False'
            assert rows(second)[0]['dead_reckoning'] == 'True'
            assert rows(second)[0]['lat_lon_valid'] == 'False'
            assert 'fix_type' not in rows(second)[0]
            wait_for(lambda: 'INVALID 3D POSITION' in (tmp_path / 'logger.txt').read_text())
        child.send_signal(signal.SIGINT)
        assert child.wait(timeout=10.0) == 0
        text = (tmp_path / 'logger.txt').read_text()
        assert 'logger shutdown' in text
        assert 'Failed to publish' not in text
        assert ('RTK_FIXED' if is_gps else 'lat_lon_valid=') in text
        assert float(rows(first)[0]['latitude_deg']) == 35.12345678912345
        assert len(rows(first)) == 2 and len(rows(second)) == 1
    finally:
        if child.poll() is None:
            child.terminate()
            child.wait(timeout=10.0)
        stdout.close()
        source.destroy_node()
        rclpy.shutdown()


@pytest.mark.skipif(os.environ.get('HERE4_RUN_DDS_TESTS') != '1',
                    reason='Set HERE4_RUN_DDS_TESTS=1 in a separate ROS domain')
def test_gps_and_ekf_record_independently_when_running_together(tmp_path):
    import rclpy
    from px4_msgs.msg import SensorGps, VehicleGlobalPosition, VehicleStatus
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

    from here4_logger.ekf_node import EkfLogger
    from here4_logger.node import GpsLogger

    assert os.environ.get('ROS_DOMAIN_ID') == '88', 'Use ROS_DOMAIN_ID=88 for this test'
    rclpy.init(args=['--ros-args', '-p', f'output_directory:={tmp_path}'])
    gps_node = GpsLogger()
    ekf_node = EkfLogger()
    source = rclpy.create_node('paired_position_test_source')
    executor = SingleThreadedExecutor()
    for node in (gps_node, ekf_node, source):
        executor.add_node(node)
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    gps_pub = source.create_publisher(SensorGps, '/fmu/out/vehicle_gps_position', qos)
    ekf_pub = source.create_publisher(
        VehicleGlobalPosition, '/fmu/out/vehicle_global_position', qos)
    status_pub = source.create_publisher(
        VehicleStatus, '/fmu/out/vehicle_status_v1', qos)

    def wait_for(predicate):
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.03)
            if predicate():
                return
        pytest.fail('Both position nodes did not reach the expected state')

    def status(armed):
        message = VehicleStatus()
        message.timestamp = 1000 if armed else 2000
        message.armed_time = 100
        message.arming_state = 2 if armed else 1
        status_pub.publish(message)

    def rows(path):
        with path.open() as stream:
            return list(csv.DictReader(stream))

    try:
        wait_for(lambda: gps_pub.get_subscription_count() == 1
                 and ekf_pub.get_subscription_count() == 1
                 and status_pub.get_subscription_count() == 2)
        status(True)
        wait_for(lambda: gps_node.recorder.recording and ekf_node.recorder.recording)
        gps = SensorGps()
        gps.latitude_deg = 35.1
        gps.longitude_deg = 51.1
        gps.altitude_msl_m = 1250.0
        gps.fix_type = 6
        gps_pub.publish(gps)
        wait_for(lambda: gps_node.recorder.rows == 1)
        assert ekf_node.recorder.rows == 0

        ekf = VehicleGlobalPosition()
        ekf.lat, ekf.lon, ekf.alt = 35.2, 51.2, 1251.0
        ekf.lat_lon_valid, ekf.alt_valid = True, True
        ekf_pub.publish(ekf)
        wait_for(lambda: ekf_node.recorder.rows == 1)
        assert gps_node.recorder.rows == 1
        gps_pub.publish(gps)
        wait_for(lambda: gps_node.recorder.rows == 2)

        status(False)
        wait_for(lambda: not gps_node.recorder.recording and not ekf_node.recorder.recording)
        gps_path, ekf_path = gps_node.recorder.path, ekf_node.recorder.path
        assert gps_path != ekf_path
        assert gps_path.name.startswith('position_')
        assert ekf_path.name.startswith('ekf_position_')
        assert len(rows(gps_path)) == 2
        assert len(rows(ekf_path)) == 1
        assert float(rows(gps_path)[0]['latitude_deg']) == 35.1
        assert float(rows(ekf_path)[0]['latitude_deg']) == 35.2
        assert rows(gps_path)[0]['armed_time_us'] == rows(ekf_path)[0]['armed_time_us']
        assert 'fix_type' in rows(gps_path)[0]
        assert 'lat_lon_valid' in rows(ekf_path)[0]
    finally:
        executor.shutdown()
        for node in (gps_node, ekf_node):
            node.close()
        for node in (gps_node, ekf_node, source):
            node.destroy_node()
        rclpy.shutdown()
