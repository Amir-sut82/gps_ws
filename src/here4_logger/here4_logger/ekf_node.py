"""Log PX4's selected fused global navigation position over XRCE-DDS."""

from px4_msgs.msg import VehicleGlobalPosition

from here4_logger.node import PositionLogger, run_logger
from here4_logger.recorder import EKF_CSV_FIELDS, ekf_values


class EkfLogger(PositionLogger):
    NODE_NAME = 'ekf_position_logger'
    SOURCE_LABEL = 'EKF'
    TOPIC_PARAMETER = 'ekf_topic'
    BASE_TOPIC = 'vehicle_global_position'
    MESSAGE_TYPE = VehicleGlobalPosition
    DEFAULT_OUTPUT_DIRECTORY = '~/ekf_logs'
    FILENAME_PREFIX = 'ekf_position'
    CSV_FIELDS = EKF_CSV_FIELDS
    sample_values = staticmethod(ekf_values)

    @staticmethod
    def quality_summary(position):
        return (
            f'lat_lon_valid={position["lat_lon_valid"]} '
            f'alt_valid={position["alt_valid"]} '
            f'dead_reckoning={position["dead_reckoning"]}')


def main(args=None):
    run_logger(EkfLogger, args)


if __name__ == '__main__':
    main()
