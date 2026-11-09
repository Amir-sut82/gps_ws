# Here4 position logger — PX4 1.17 / ROS 2 Humble

This workspace receives GPS and arming state already forwarded by your
Micro XRCE-DDS Agent on the Orange Pi. It prints GPS data in the terminal and
writes a separate CSV for each confirmed armed session. The logger has no
vehicle-command publishers and does not open the serial port.

Data path:

```text
Here4 → Pixhawk 6C Mini / PX4 → serial → MicroXRCEAgent → ROS 2 → terminal + CSV
```

## Build on the Orange Pi

The official `px4_msgs` source for `release/1.17` is included in `src/px4_msgs`.
Its exact revision is recorded in `px4_msgs.repos`. Build on the Orange Pi even
if you copied this workspace from another computer: compiled files are specific
to the computer architecture. Copy `src/`, `scripts/`, and `px4_msgs.repos`;
exclude `build/`, `install/`, and `log/` when transferring to another machine.

With ROS 2 Humble and colcon installed:

```bash
cd ~/gps_ws
source /opt/ros/humble/setup.bash
export CMAKE_BUILD_PARALLEL_LEVEL=2
export MAKEFLAGS=-j2
colcon build --symlink-install --executor sequential \
  --packages-up-to here4_logger --cmake-args -DBUILD_TESTING=OFF
```

If dependencies are missing, install them with rosdep and repeat the build:

```bash
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
```

Use the included PX4 1.17 messages when sourcing this workspace. Sourcing
another workspace containing a different `px4_msgs` afterward can select
incompatible message definitions.

## Run

In terminal 1 on the Orange Pi, run your existing agent command. Keep one agent
running for this serial link; leave it running if it is already connected:

```bash
MicroXRCEAgent serial --dev /dev/ttyS6 -b 921600
```

In terminal 2:

```bash
cd ~/gps_ws
./scripts/start_logger.sh
```

Or launch it directly:

```bash
source /opt/ros/humble/setup.bash
source ~/gps_ws/install/setup.bash
ros2 launch here4_logger logger.launch.py
```

The terminal displays data at 1 Hz, including latitude, longitude, MSL altitude,
GPS fix/RTK state, satellites, accuracy estimates, ground speed, arming state,
and data age. Logging records **every GPS message received while armed**,
independently of the display rate.

## Logging behavior

- A confirmed armed state starts a new CSV, even if the logger starts while
  the vehicle is already armed. Repeated armed messages keep the same file.
- Disarming closes and syncs the file. The terminal prints its full path and
  number of samples. The next arm creates a different file.
- An arm request rejected by PX4 creates no session. This tracks actual state,
  not the RC switch or command request.
- Ctrl+C closes and saves an active file. GPS samples already received before
  a confirmed arm are not added retroactively.
- Missing or stale GPS/status data is marked in the terminal. If status stops
  while armed, incoming GPS continues to be recorded with `status_fresh=False`.
  A changed PX4 `armed_time` starts a new file if a disarm/rearm transition was
  missed. Data and entire sessions lost during a DDS interruption cannot be
  recovered by this receiver.
- Invalid fixes are retained with fix and validity fields for later filtering.
- Each row is flushed to the OS immediately. Files are synced to storage every
  second and on close. Abrupt power loss can still lose recent writes.

Files are written to `~/gps_logs/`, for example:

```text
~/gps_logs/position_20261109T120000_123456Z_a1b2c3d4.csv
```

Filenames and `received_utc` use UTC. `received_utc` is the Orange Pi receive
time, so its system clock should be correct. The original PX4 timestamps and
GPS UTC timestamp are also preserved. No conversion between those clock
domains is assumed. A GPS UTC timestamp of zero means it is unavailable.

Important CSV columns:

| Columns | Meaning / units |
| --- | --- |
| `latitude_deg`, `longitude_deg` | GPS coordinates in degrees; original float64 precision |
| `altitude_msl_m`, `altitude_ellipsoid_m` | Meters above mean sea level / WGS84 ellipsoid |
| `fix_type`, `fix_name` | 3 = 3D, 4 = DGPS, 5 = RTK float, 6 = RTK fixed |
| `position_3d_valid` | 3D-or-better fix and finite coordinates/altitude within coordinate bounds; does not certify accuracy |
| `eph`, `epv` | Horizontal and vertical accuracy estimates in meters |
| `vel_m_s`, `vel_n_m_s`, `vel_e_m_s`, `vel_d_m_s` | Speed / North-East-Down velocity in m/s; Down is positive downward |
| `vel_ned_valid` | Whether the receiver considers the NED velocity valid |
| `timestamp`, `timestamp_sample`, `status_timestamp_us`, `armed_time_us` | Original PX4 timestamps in microseconds |
| `time_utc_usec` | Receiver GPS UTC timestamp in microseconds |
| `session_elapsed_s`, `status_age_s`, `status_fresh` | Receive-time session elapsed time and status freshness |
| `device_id`, `satellites_used`, `rtcm_*` | GPS device, satellite count, and correction diagnostics |
| `arming_state`, `nav_state`, `failsafe` | Most recently received PX4 status |

Additional receiver fields, including heading, dilution of precision, jamming,
spoofing, and authentication diagnostics, are retained under their original
PX4 names. These are GPS positions from the PX4 GPS pipeline, not the fused
EKF `vehicle_global_position` estimate. If multiple GPS units are configured,
PX4 may select or blend their data; this node does not select a receiver.

## Configuration and topic checks

Edit `src/here4_logger/config/logger.yaml` before starting. With
`--symlink-install`, edits are used on the next launch. Configuration values
are read at startup. Launch arguments override corresponding YAML settings:

```bash
./scripts/start_logger.sh output_directory:=/home/orangepi/flight_logs
./scripts/start_logger.sh px4_namespace:=px4_1
```

Default topic names are derived from the installed message version:

| Topic | Message |
| --- | --- |
| `/fmu/out/vehicle_gps_position` | `px4_msgs/msg/SensorGps` |
| `/fmu/out/vehicle_status_v1` | `px4_msgs/msg/VehicleStatus` (version 1) |

Both subscribers use best-effort, volatile QoS compatible with PX4. If you see
topics but no data, check the actual names, types, publishers, and QoS after
sourcing this workspace:

```bash
source /opt/ros/humble/setup.bash
source ~/gps_ws/install/setup.bash
ros2 topic list -t
ros2 topic info /fmu/out/vehicle_gps_position --verbose
ros2 topic info /fmu/out/vehicle_status_v1 --verbose
ros2 topic echo /fmu/out/vehicle_gps_position --qos-reliability best_effort
```

For a firmware build with different topic names, override them explicitly
after confirming that their message definitions match:

```bash
./scripts/start_logger.sh \
  gps_topic:=/fmu/out/vehicle_gps_position \
  status_topic:=/fmu/out/vehicle_status_v1
```

Keep `ROS_DOMAIN_ID` and the DDS environment consistent with the environment
where your agent's topics are already visible. Keep your working agent/PX4
serial configuration. This workspace needs neither MAVROS nor another serial
connection.

## Tests

Recorder tests without hardware or ROS discovery:

```bash
cd ~/gps_ws
PYTHONPATH=src/here4_logger python3 -m pytest -q \
  src/here4_logger/test/test_recorder.py
```

After building, run the optional DDS test in its isolated domain. It publishes
synthetic GPS/status data on domain 88 and checks CSV rotation, terminal
output, precision, and saving on Ctrl+C:

```bash
source /opt/ros/humble/setup.bash
source ~/gps_ws/install/setup.bash
ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1 HERE4_RUN_DDS_TESTS=1 \
  python3 -m pytest -q src/here4_logger/test/test_dds.py
```

Reference definitions: [PX4 1.17 SensorGps](https://github.com/PX4/px4_msgs/blob/release/1.17/msg/SensorGps.msg),
[VehicleStatus](https://github.com/PX4/px4_msgs/blob/release/1.17/msg/VehicleStatus.msg),
[DDS topic mapping](https://github.com/PX4/PX4-Autopilot/blob/release/1.17/src/modules/uxrce_dds_client/dds_topics.yaml),
and [PX4 ROS 2 subscriber QoS](https://docs.px4.io/main/en/ros2/user_guide#ros-2-subscriber-qos-settings).
