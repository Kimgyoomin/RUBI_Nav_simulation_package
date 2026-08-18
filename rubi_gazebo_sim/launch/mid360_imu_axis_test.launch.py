import os

from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.substitutions import EnvironmentVariable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    sim_share = get_package_share_directory('rubi_gazebo_sim')
    description_share = get_package_share_directory('rubi_description')
    gazebo_share = get_package_share_directory('gazebo_ros')
    gazebo_plugins_prefix = get_package_prefix('gazebo_plugins')
    world = os.path.join(
        sim_share, 'worlds', 'mid360_imu_axis_test.world')
    xacro_file = os.path.join(
        description_share, 'urdf', 'test',
        'mid360_imu_axis_test.urdf.xacro')
    robot_description = ParameterValue(
        Command(['xacro', ' ', xacro_file]), value_type=str)

    return LaunchDescription([
        SetEnvironmentVariable(
            'GAZEBO_PLUGIN_PATH',
            [
                os.path.join(gazebo_plugins_prefix, 'lib'),
                os.pathsep,
                EnvironmentVariable('GAZEBO_PLUGIN_PATH', default_value=''),
            ],
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True,
            }],
            output='screen',
        ),
        Node(
            package='rubi_gazebo_sim',
            executable='mid360_frame_adapter_node',
            name='mid360_frame_adapter',
            parameters=[{
                'apply_roll_180': True,
                'input_imu_topic': '/livox/imu_raw',
                'output_imu_topic': '/livox/imu',
                'output_frame_id': 'livox_frame',
                'use_sim_time': True,
            }],
            output='screen',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_share, 'launch', 'gzserver.launch.py')),
            launch_arguments={
                'world': world,
                'pause': 'false',
                'verbose': 'true',
                'server_required': 'true',
            }.items(),
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-topic', '/robot_description',
                '-entity', 'mid360_imu_axis_test',
                '-z', '1.0',
            ],
            output='screen',
        ),
    ])
