from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


OVERRIDES = ('output_directory', 'px4_namespace', 'gps_topic', 'status_topic')


def launch_logger(context):
    overrides = {}
    for name in OVERRIDES:
        value = LaunchConfiguration(name).perform(context)
        if value:
            overrides[name] = ParameterValue(value, value_type=str)
    return [Node(
        package='here4_logger', executable='gps_logger', name='here4_logger',
        output='screen', emulate_tty=True,
        parameters=[LaunchConfiguration('config'), overrides],
    )]


def generate_launch_description():
    config = get_package_share_directory('here4_logger') + '/config/logger.yaml'
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=config),
        DeclareLaunchArgument('output_directory', default_value=''),
        DeclareLaunchArgument('px4_namespace', default_value=''),
        DeclareLaunchArgument('gps_topic', default_value=''),
        DeclareLaunchArgument('status_topic', default_value=''),
        OpaqueFunction(function=launch_logger),
    ])
