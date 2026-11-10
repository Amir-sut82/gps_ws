import csv
import math
from types import SimpleNamespace

import pytest

from here4_logger.recorder import EKF_CSV_FIELDS, ekf_values, PositionRecorder


def estimate(**changes):
    values = dict(
        timestamp=1000, timestamp_sample=900,
        lat=35.12345678912345, lon=51.12345678912345,
        alt=1250.125, alt_ellipsoid=1275.25,
        lat_lon_valid=True, alt_valid=True, eph=0.04, epv=0.08,
        dead_reckoning=False, delta_alt=0.25, delta_terrain=0.5,
        lat_lon_reset_counter=1, alt_reset_counter=2, terrain_reset_counter=3,
        terrain_alt=1200.0, terrain_alt_valid=True,
    )
    values.update(changes)
    return SimpleNamespace(**values)


def status(armed, armed_time=100):
    return SimpleNamespace(
        arming_state=2 if armed else 1, armed_time=armed_time,
        timestamp=200, nav_state=3, failsafe=False)


def rows(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def test_ekf_coordinates_keep_precision_units_and_diagnostics():
    message = estimate()
    values = ekf_values(message)
    assert values['latitude_deg'] == message.lat
    assert values['longitude_deg'] == message.lon
    assert values['altitude_msl_m'] == message.alt
    assert values['altitude_ellipsoid_m'] == message.alt_ellipsoid
    assert values['position_3d_valid']
    assert values['lat_lon_reset_counter'] == 1
    assert values['alt_reset_counter'] == 2
    assert values['terrain_reset_counter'] == 3
    assert values['eph'] == 0.04
    assert values['epv'] == 0.08
    assert 'fix_type' not in values


@pytest.mark.parametrize('changes', [
    {'lat_lon_valid': False}, {'alt_valid': False},
    {'lat': math.nan}, {'lat': 91.0}, {'lon': -181.0}, {'alt': math.inf},
])
def test_ekf_validity_requires_both_px4_flags_and_finite_coordinates(changes):
    assert not ekf_values(estimate(**changes))['position_3d_valid']


def test_dead_reckoning_flag_is_preserved_separately_from_validity():
    values = ekf_values(estimate(dead_reckoning=True))
    assert values['dead_reckoning']
    assert values['position_3d_valid']


def test_ekf_has_its_own_log_schema_and_arm_disarm_sessions(tmp_path):
    recorder = PositionRecorder(
        tmp_path, fieldnames=EKF_CSV_FIELDS, filename_prefix='ekf_position')
    valid = ekf_values(estimate())
    assert not recorder.write_position(valid)
    first = recorder.update_status(status(True))[0].path
    assert recorder.update_status(status(True)) == []
    recorder.write_position(valid)
    recorder.write_position(ekf_values(estimate(lat_lon_valid=False)))
    saved = recorder.update_status(status(False))[0]
    assert saved.rows == 2
    assert first.name.startswith('ekf_position_')
    assert rows(first)[1]['position_3d_valid'] == 'False'
    assert float(rows(first)[0]['latitude_deg']) == 35.12345678912345
    assert float(rows(first)[0]['altitude_msl_m']) == 1250.125
    assert set(rows(first)[0]) == set(EKF_CSV_FIELDS)
    assert not recorder.write_position(valid)

    second = recorder.update_status(status(True, 500))[0].path
    recorder.write_position(valid)
    shutdown = recorder.close()
    assert shutdown.reason == 'shutdown' and shutdown.rows == 1
    assert first != second
    assert len(rows(second)) == 1


def test_ekf_keeps_recording_with_stale_arming_status(tmp_path):
    now = [0.0]
    recorder = PositionRecorder(
        tmp_path, monotonic_clock=lambda: now[0],
        fieldnames=EKF_CSV_FIELDS, filename_prefix='ekf_position')
    recorder.update_status(status(True))
    now[0] = 4.5
    recorder.write_position(ekf_values(estimate()))
    saved = recorder.close()
    row = rows(saved.path)[0]
    assert row['status_fresh'] == 'False'
    assert float(row['status_age_s']) == 4.5
