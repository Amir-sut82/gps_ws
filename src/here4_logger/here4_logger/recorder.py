"""CSV recording logic, independent of ROS and serial hardware."""

import csv
from dataclasses import dataclass
from datetime import datetime, timezone
import math
import os
from pathlib import Path
import time
from uuid import uuid4


FIX_NAMES = {
    0: 'NO_FIX', 1: 'NO_FIX', 2: '2D', 3: '3D',
    4: 'DGPS', 5: 'RTK_FLOAT', 6: 'RTK_FIXED', 8: 'EXTRAPOLATED',
}

# These fields and units come from PX4 release/1.17 SensorGps.msg.
GPS_FIELDS = (
    'timestamp', 'timestamp_sample', 'time_utc_usec', 'device_id',
    'latitude_deg', 'longitude_deg', 'altitude_msl_m', 'altitude_ellipsoid_m',
    'fix_type', 'satellites_used', 'eph', 'epv', 'hdop', 'vdop',
    'vel_m_s', 'vel_n_m_s', 'vel_e_m_s', 'vel_d_m_s', 'vel_ned_valid',
    's_variance_m_s', 'c_variance_rad', 'cog_rad',
    'heading', 'heading_offset', 'heading_accuracy',
    'rtcm_injection_rate', 'selected_rtcm_instance', 'rtcm_crc_failed',
    'rtcm_msg_used', 'jamming_state', 'jamming_indicator', 'spoofing_state',
    'authentication_state', 'system_error',
)

CSV_FIELDS = (
    'received_utc', 'session_elapsed_s', 'status_timestamp_us',
    'armed_time_us', 'status_age_s', 'status_fresh', 'arming_state',
    'nav_state', 'failsafe', 'fix_name', 'position_3d_valid',
) + GPS_FIELDS


def utc_now():
    return datetime.now(timezone.utc)


def gps_values(message):
    """Keep float64 coordinates intact; retain invalid fixes with quality flags."""
    values = {name: getattr(message, name) for name in GPS_FIELDS}
    values['fix_name'] = FIX_NAMES.get(values['fix_type'], 'UNKNOWN')
    lat, lon = values['latitude_deg'], values['longitude_deg']
    values['position_3d_valid'] = (
        values['fix_type'] in (3, 4, 5, 6)
        and math.isfinite(lat) and -90.0 <= lat <= 90.0
        and math.isfinite(lon) and -180.0 <= lon <= 180.0
        and math.isfinite(values['altitude_msl_m'])
    )
    return values


@dataclass(frozen=True)
class SessionEvent:
    kind: str
    path: Path
    rows: int
    reason: str


class PositionRecorder:
    """Write GPS arrivals while the latest received VehicleStatus says armed."""

    def __init__(self, output_directory, status_timeout_s=3.0,
                 monotonic_clock=time.monotonic, wall_clock=utc_now):
        self.output_directory = Path(output_directory).expanduser().resolve()
        self.output_directory.mkdir(parents=True, exist_ok=True)
        self.status_timeout_s = status_timeout_s
        self.monotonic = monotonic_clock
        self.wall_clock = wall_clock
        self.armed = None
        self.status = None
        self.status_received_at = None
        self.path = None
        self.rows = 0
        self._file = None
        self._writer = None
        self._started_at = None
        self._armed_time = 0

    def update_status(self, message, armed_state=2):
        """Start once per arm, including startup armed or a missed rearm edge."""
        events = []
        armed = message.arming_state == armed_state
        armed_time = int(message.armed_time)
        if self._file is not None:
            if not armed:
                events.append(self.close('disarmed'))
            elif armed_time and self._armed_time and armed_time != self._armed_time:
                events.append(self.close('new arming timestamp'))

        self.armed = armed
        self.status = message
        self.status_received_at = self.monotonic()
        if armed and self._file is None:
            self._armed_time = armed_time
            events.append(self._start())
        elif armed_time:
            self._armed_time = armed_time
        return events

    def _start(self):
        stamp = self.wall_clock().strftime('%Y%m%dT%H%M%S_%fZ')
        path = self.output_directory / f'position_{stamp}_{uuid4().hex[:8]}.csv'
        # Exclusive creation prevents an old flight log from being overwritten.
        stream = path.open('x', newline='', encoding='utf-8')
        try:
            writer = csv.DictWriter(stream, fieldnames=CSV_FIELDS)
            writer.writeheader()
            stream.flush()
            os.fsync(stream.fileno())
        except BaseException:
            stream.close()
            raise
        self.path, self._file, self._writer = path, stream, writer
        self.rows = 0
        self._started_at = self.monotonic()
        return SessionEvent('started', path, 0, 'armed')

    @property
    def recording(self):
        return self._file is not None

    def status_age(self):
        if self.status_received_at is None:
            return math.inf
        return max(0.0, self.monotonic() - self.status_received_at)

    def write_gps(self, values):
        if self._file is None:
            return False
        age = self.status_age()
        row = {
            'received_utc': self.wall_clock().isoformat(timespec='microseconds'),
            'session_elapsed_s': round(self.monotonic() - self._started_at, 6),
            'status_timestamp_us': self.status.timestamp,
            'armed_time_us': self._armed_time,
            'status_age_s': round(age, 6),
            'status_fresh': age <= self.status_timeout_s,
            'arming_state': self.status.arming_state,
            'nav_state': self.status.nav_state,
            'failsafe': self.status.failsafe,
            **values,
        }
        self._writer.writerow(row)
        # Each sample is immediately visible to another process reading the CSV.
        self._file.flush()
        self.rows += 1
        return True

    def flush(self):
        if self._file is not None:
            self._file.flush()
            os.fsync(self._file.fileno())

    def close(self, reason='shutdown'):
        if self._file is None:
            return None
        event = SessionEvent('saved', self.path, self.rows, reason)
        try:
            self.flush()
        finally:
            self._file.close()
            self._file = None
            self._writer = None
        return event
