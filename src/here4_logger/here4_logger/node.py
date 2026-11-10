"""Shared PX4 position monitor and the Here4 GPS logger entry point."""

import math
import time

from px4_msgs.msg import SensorGps, VehicleStatus
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
)

from here4_logger.recorder import CSV_FIELDS, gps_values, PositionRecorder


def px4_topic(name, message_type, namespace=''):
    version = getattr(message_type, 'MESSAGE_VERSION', 0)
    suffix = f'_v{version}' if version else ''
    prefix = namespace.strip('/')
    return f'/{prefix + "/" if prefix else ""}fmu/out/{name}{suffix}'


class PositionLogger(Node):
    """Receive one position source and save it using confirmed PX4 arming state."""

    def __init__(self):
        super().__init__(self.NODE_NAME)
        self.declare_parameter('output_directory', self.DEFAULT_OUTPUT_DIRECTORY)
        self.declare_parameter('px4_namespace', '')
        self.declare_parameter(self.TOPIC_PARAMETER, 'auto')
        self.declare_parameter('status_topic', 'auto')
        self.declare_parameter('print_rate_hz', 1.0)
        self.declare_parameter('stale_timeout_s', 3.0)
        self.declare_parameter('flush_interval_s', 1.0)
        # Keep accidental runtime parameter edits from appearing to take effect.
        self.add_on_set_parameters_callback(self._reject_parameter_changes)
        namespace = self.get_parameter('px4_namespace').value
        self.position_topic = self.get_parameter(self.TOPIC_PARAMETER).value
        self.status_topic = self.get_parameter('status_topic').value
        if self.position_topic == 'auto':
            self.position_topic = px4_topic(self.BASE_TOPIC, self.MESSAGE_TYPE, namespace)
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
        self.recorder = PositionRecorder(
            output_directory, self.stale_timeout,
            fieldnames=self.CSV_FIELDS, filename_prefix=self.FILENAME_PREFIX)
        self.latest_position = None
        self.position_received_at = None
        self.received_samples = 0

        # BEST_EFFORT + VOLATILE accepts PX4 best-effort or reliable writers
        # and does not replay historical arming transitions on startup.
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=100,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.position_subscription = self.create_subscription(
            self.MESSAGE_TYPE, self.position_topic, self._on_position, qos)
        self.status_subscription = self.create_subscription(
            VehicleStatus, self.status_topic, self._on_status, qos)
        self.print_timer = self.create_timer(1.0 / print_rate, self._print_status)
        self.flush_timer = self.create_timer(flush_interval, self.recorder.flush)
        self.get_logger().info(
            f'{self.SOURCE_LABEL}: {self.position_topic} | status: {self.status_topic}')
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

    def _on_position(self, message):
        self.latest_position = self.sample_values(message)
        self.position_received_at = time.monotonic()
        self.received_samples += 1
        self.recorder.write_position(self.latest_position)

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
        if self.latest_position is None:
            text = f'{state} | waiting for {self.SOURCE_LABEL} on {self.position_topic}'
            if self.recorder.status is None:
                text += f' | waiting for status on {self.status_topic}'
            self.get_logger().info(text)
            return

        position = self.latest_position
        age = max(0.0, time.monotonic() - self.position_received_at)
        stale = f' [{self.SOURCE_LABEL} STALE]' if age > self.stale_timeout else ''
        validity = 'valid 3D' if position['position_3d_valid'] else 'INVALID 3D POSITION'
        self.get_logger().info(
            f'{state} | lat={position["latitude_deg"]:.9f} '
            f'lon={position["longitude_deg"]:.9f} '
            f'alt_MSL={position["altitude_msl_m"]:.3f} m | '
            f'{self.quality_summary(position)} {validity} | '
            f'hAcc={position["eph"]:.3f} m vAcc={position["epv"]:.3f} m | '
            f'{self.SOURCE_LABEL} age={age:.1f}s{stale} '
            f'status age={status_age:.1f}s | samples={self.received_samples} '
            f'CSV rows={self.recorder.rows}')

    def close(self):
        self._report_event(self.recorder.close('logger shutdown'))


class GpsLogger(PositionLogger):
    NODE_NAME = 'here4_logger'
    SOURCE_LABEL = 'GPS'
    TOPIC_PARAMETER = 'gps_topic'
    BASE_TOPIC = 'vehicle_gps_position'
    MESSAGE_TYPE = SensorGps
    DEFAULT_OUTPUT_DIRECTORY = '~/gps_logs'
    FILENAME_PREFIX = 'position'
    CSV_FIELDS = CSV_FIELDS
    sample_values = staticmethod(gps_values)

    @staticmethod
    def quality_summary(position):
        return (
            f'fix={position["fix_name"]} ({position["fix_type"]}) '
            f'sats={position["satellites_used"]} '
            f'speed={position["vel_m_s"]:.3f} m/s')


def run_logger(node_class, args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = node_class()
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


def main(args=None):
    run_logger(GpsLogger, args)


if __name__ == '__main__':
    main()
