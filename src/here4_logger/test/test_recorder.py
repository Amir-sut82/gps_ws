import csv
from datetime import datetime, timezone
import math
from types import SimpleNamespace

import pytest

from here4_logger.recorder import GPS_FIELDS, gps_values, PositionRecorder


class Clock:
    now = 0.0

    def monotonic(self):
        return self.now

    def utc(self):
        return datetime(2026, 11, 9, tzinfo=timezone.utc)


def status(armed, armed_time=100, timestamp=200):
    return SimpleNamespace(
        arming_state=2 if armed else 1, armed_time=armed_time,
        timestamp=timestamp, nav_state=3, failsafe=False)


def gps(fix=6, latitude=35.12345678912345):
    fields = dict.fromkeys(GPS_FIELDS, 0)
    fields.update(
        latitude_deg=latitude, longitude_deg=51.12345678912345,
        altitude_msl_m=1234.123456789, altitude_ellipsoid_m=1260.0,
        fix_type=fix, timestamp=500, satellites_used=24)
    return SimpleNamespace(**fields)


def read_rows(path):
    with path.open(newline='') as stream:
        return list(csv.DictReader(stream))


def test_only_received_gps_while_armed_is_logged(tmp_path):
    recorder = PositionRecorder(tmp_path)
    values = gps_values(gps())
    assert not recorder.write_gps(values)
    assert recorder.update_status(status(False)) == []
    assert not recorder.write_gps(values)
    assert list(tmp_path.iterdir()) == []

    started = recorder.update_status(status(True))
    assert len(started) == 1 and started[0].kind == 'started'
    assert recorder.write_gps(values)
    # Rows can be read while the session is still open.
    assert len(read_rows(started[0].path)) == 1
    saved = recorder.update_status(status(False))
    assert saved[0].reason == 'disarmed' and saved[0].rows == 1
    assert not recorder.write_gps(values)
    assert recorder.close() is None


def test_repeat_armed_messages_keep_one_file_and_rearm_creates_another(tmp_path):
    recorder = PositionRecorder(tmp_path)
    first = recorder.update_status(status(True))[0].path
    for _ in range(5):
        assert recorder.update_status(status(True)) == []
        recorder.write_gps(gps_values(gps()))
    recorder.update_status(status(False))
    second = recorder.update_status(status(True, armed_time=200))[0].path
    recorder.write_gps(gps_values(gps()))
    recorder.close()
    assert first != second
    assert len(read_rows(first)) == 5
    assert len(read_rows(second)) == 1


def test_starting_already_armed_creates_a_session(tmp_path):
    recorder = PositionRecorder(tmp_path)
    assert recorder.armed is None
    event = recorder.update_status(status(True))[0]
    recorder.close()
    assert event.path.exists()
    assert read_rows(event.path) == []


def test_changed_armed_time_recovers_missed_disarm_rearm(tmp_path):
    recorder = PositionRecorder(tmp_path)
    first = recorder.update_status(status(True))[0].path
    recorder.write_gps(gps_values(gps()))
    events = recorder.update_status(status(True, armed_time=900))
    assert [event.kind for event in events] == ['saved', 'started']
    assert events[0].reason == 'new arming timestamp'
    assert events[1].path != first
    recorder.close()


def test_shutdown_keeps_partial_session_and_coordinate_precision(tmp_path):
    recorder = PositionRecorder(tmp_path)
    recorder.update_status(status(True))
    message = gps()
    recorder.write_gps(gps_values(message))
    event = recorder.close()
    row = read_rows(event.path)[0]
    assert event.reason == 'shutdown'
    assert float(row['latitude_deg']) == message.latitude_deg
    assert float(row['longitude_deg']) == message.longitude_deg
    assert float(row['altitude_msl_m']) == message.altitude_msl_m
    assert row['fix_name'] == 'RTK_FIXED'
    assert row['position_3d_valid'] == 'True'
    assert row['received_utc'].endswith('+00:00')


@pytest.mark.parametrize('fix, latitude', [
    (1, 35.0), (2, 35.0), (8, 35.0), (6, math.nan), (6, 91.0),
])
def test_invalid_positions_are_retained_and_flagged(tmp_path, fix, latitude):
    recorder = PositionRecorder(tmp_path)
    recorder.update_status(status(True))
    values = gps_values(gps(fix, latitude))
    assert not values['position_3d_valid']
    recorder.write_gps(values)
    event = recorder.close()
    assert len(read_rows(event.path)) == 1
    assert read_rows(event.path)[0]['position_3d_valid'] == 'False'


def test_status_loss_continues_logging_with_stale_marker(tmp_path):
    clock = Clock()
    recorder = PositionRecorder(tmp_path, 3.0, clock.monotonic, clock.utc)
    assert math.isinf(recorder.status_age())
    recorder.update_status(status(True))
    clock.now = 4.5
    recorder.write_gps(gps_values(gps()))
    event = recorder.close()
    row = read_rows(event.path)[0]
    assert row['status_fresh'] == 'False'
    assert float(row['status_age_s']) == 4.5
    assert float(row['session_elapsed_s']) == 4.5


def test_same_wall_clock_never_overwrites_a_session(tmp_path):
    clock = Clock()
    recorder = PositionRecorder(tmp_path, wall_clock=clock.utc)
    paths = []
    for _ in range(3):
        paths.append(recorder.update_status(status(True))[0].path)
        recorder.update_status(status(False))
    assert len(set(paths)) == 3
    assert len(list(tmp_path.glob('*.csv'))) == 3
