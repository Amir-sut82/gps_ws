"""Receive GPS and arming state from PX4; never publish vehicle commands."""

import math
import time

from px4_msgs.msg import SensorGps, VehicleStatus
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
)

from here4_logger.recorder import gps_values, PositionRecorder


def px4_topic(name, message_type, namespace=''):
    version = getattr(message_type, 'MESSAGE_VERSION', 0)
    suffix = f'_v{version}' if version else ''
    prefix = namespace.strip('/')
    return f'/{prefix + "/" if prefix else ""}fmu/out/{name}{suffix}'


class GpsLogger(Node):
    def __init__(self):
        super().__init__('here4_logger')
        self.declare_parameter('output_directory', '~/gps_logs')
        self.declare_parameter('px4_namespace', '')
        self.declare_parameter('gps_topic', 'auto')
        self.declare_parameter('status_topic', 'auto')
        self.declare_parameter('print_rate_hz', 1.0)
        self.declare_parameter('stale_timeout_s', 3.0)
        self.declare_parameter('flush_interval_s', 1.0)
        # Keep accidental runtime parameter edits from appearing to take effect.
        self.add_on_set_parameters_callback(self._reject_parameter_changes)
        namespace = self.get_parameter('px4_namespace').value
        self.gps_topic = self.get_parameter('gps_topic').value
        self.status_topic = self.get_parameter('status_topic').value
        if self.gps_topic == 'auto':
            self.gps_topic = px4_topic('vehicle_gps_position', SensorGps, namespace)
        if self.status_topic == 'auto':
            self.status_topic = px4_topic('vehicle_status', VehicleStatus, namespace)

        print_rate = float(self.get_parameter('print_rate_hz').value)
        self.stale_timeout = float(self.get_parameter('stale_timeout_s').value)
        flush_interval = float(self.get_parameter('flush_interval_s').value)
        for name, value in (
            ('print_rate_hz', print_rate), ('stale_timeout_s', self.stale_timeout),
            ('flush_interval_s', flush_interval),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f'{name} must be finite and greater than zero')
        output_directory = self.get_parameter('output_directory').value
        if not output_directory.strip():
            raise ValueError('output_directory must not be empty')
        self.recorder = PositionRecorder(output_directory, self.stale_timeout)
        self.latest_gps = None
        self.gps_received_at = None
        self.received_samples = 0

        # BEST_EFFORT + VOLATILE accepts PX4 best-effort or reliable writers
        # and does not replay historical arming transitions on startup.
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=100,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.gps_subscription = self.create_subscription(
            SensorGps, self.gps_topic, self._on_gps, qos)
        self.status_subscription = self.create_subscription(
            VehicleStatus, self.status_topic, self._on_status, qos)
        self.print_timer = self.create_timer(1.0 / print_rate, self._print_status)
        self.flush_timer = self.create_timer(flush_interval, self.recorder.flush)
        self.get_logger().info(
            f'GPS: {self.gps_topic} | status: {self.status_topic}')
        self.get_logger().info(
            f'CSV directory: {self.recorder.output_directory} | '
            'waiting for PX4 data; a new file starts on each confirmed arm.')

    @staticmethod
    def _reject_parameter_changes(parameters):
        from rcl_interfaces.msg import SetParametersResult
        return SetParametersResult(
            successful=False, reason='Restart the logger to change its configuration.')

    def _on_status(self, message):
        events = self.recorder.update_status(message, VehicleStatus.ARMING_STATE_ARMED)
        for event in events:
            self._report_event(event)

    def _on_gps(self, message):
        self.latest_gps = gps_values(message)
        self.gps_received_at = time.monotonic()
        self.received_samples += 1
        self.recorder.write_gps(self.latest_gps)

    def _report_event(self, event):
        if event is not None:
            text = (
                f'CSV {event.kind}: {event.path} | rows={event.rows} | {event.reason}')
            if self.context.ok():
                self.get_logger().info(text)
            else:
                # Humble shuts down the ROS context before spin returns on SIGINT.
                print(text, flush=True)

    def _print_status(self):
        status_age = self.recorder.status_age()
        if self.recorder.armed is None:
            state = 'ARM_STATE_UNKNOWN'
        else:
            state = 'ARMED' if self.recorder.armed else 'DISARMED'
        if status_age > self.stale_timeout:
            state += ' [STATUS MISSING/STALE]'
        if self.latest_gps is None:
            text = f'{state} | waiting for GPS on {self.gps_topic}'
            if self.recorder.status is None:
                text += f' | waiting for status on {self.status_topic}'
            self.get_logger().info(text)
            return

        gps = self.latest_gps
        age = max(0.0, time.monotonic() - self.gps_received_at)
        stale = ' [GPS STALE]' if age > self.stale_timeout else ''
        validity = 'valid 3D' if gps['position_3d_valid'] else 'INVALID 3D POSITION'
        self.get_logger().info(
            f'{state} | lat={gps["latitude_deg"]:.9f} '
            f'lon={gps["longitude_deg"]:.9f} '
            f'alt_MSL={gps["altitude_msl_m"]:.3f} m | '
            f'fix={gps["fix_name"]} ({gps["fix_type"]}) '
            f'sats={gps["satellites_used"]} {validity} | '
            f'hAcc={gps["eph"]:.3f} m vAcc={gps["epv"]:.3f} m '
            f'speed={gps["vel_m_s"]:.3f} m/s | GPS age={age:.1f}s{stale} '
            f'status age={status_age:.1f}s | samples={self.received_samples} '
            f'CSV rows={self.recorder.rows}')

    def close(self):
        self._report_event(self.recorder.close('logger shutdown'))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = GpsLogger()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            if node is not None:
                try:
                    node.close()
                finally:
                    node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()
