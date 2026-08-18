import glob
import os
import shutil
import tempfile

from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    sim_share = get_package_share_directory('rubi_gazebo_sim')
    description_share = get_package_share_directory('rubi_description')
    livox_prefix = get_package_prefix('ros2_livox_simulation')
    livox_share = get_package_share_directory('ros2_livox_simulation')
    gazebo_plugins_prefix = get_package_prefix('gazebo_plugins')
    gazebo_share = get_package_share_directory('gazebo_ros')

    world = os.path.join(sim_share, 'worlds', 'rubi_mid360_imu.world')
    xacro_file = os.path.join(
        description_share, 'urdf', 'RUBI_mid360_imu.urdf.xacro')
    rviz_config = os.path.join(sim_share, 'config', 'rubi_mid360.rviz')
    system_model_paths = sorted(glob.glob('/usr/share/gazebo-*/models'))
    model_paths = [os.path.join(sim_share, 'models'), *system_model_paths]
    robot_description = ParameterValue(
        Command(['xacro', ' ', xacro_file]), value_type=str)
    shutil.copyfile(
        os.path.join(livox_share, 'scan_mode', 'mid360.csv'),
        os.path.join(tempfile.gettempdir(), 'rubi_mid360_imu_mid360.csv'),
    )

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=world),
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('paused', default_value='false'),
        DeclareLaunchArgument('verbose', default_value='true'),
        DeclareLaunchArgument('namespace', default_value='/rubi_mid360'),
        SetEnvironmentVariable(
            'GAZEBO_MODEL_PATH',
            [
                os.pathsep.join(model_paths),
                os.pathsep,
                EnvironmentVariable('GAZEBO_MODEL_PATH', default_value=''),
            ],
        ),
        SetEnvironmentVariable(
            'GAZEBO_PLUGIN_PATH',
            [
                os.path.join(livox_prefix, 'lib'),
                os.pathsep,
                os.path.join(gazebo_plugins_prefix, 'lib'),
                os.pathsep,
                EnvironmentVariable('GAZEBO_PLUGIN_PATH', default_value=''),
            ],
        ),
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=LaunchConfiguration('namespace'),
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
            output='screen',
        ),
        Node(
            package='rubi_gazebo_sim',
            executable='mid360_frame_adapter_node',
            name='mid360_frame_adapter',
            parameters=[{
                'apply_roll_180': True,
                'input_lidar_topic': '/livox/lidar_raw',
                'output_lidar_topic': '/livox/lidar',
                'input_pointcloud2_topic':
                    '/livox/lidar_raw_PointCloud2',
                'output_pointcloud2_topic': '/livox/lidar_PointCloud2',
                'input_imu_topic': '/livox/imu_raw',
                'output_imu_topic': '/livox/imu',
                'output_frame_id': 'livox_frame',
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
            output='screen',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_share, 'launch', 'gzserver.launch.py')),
            launch_arguments={
                'world': LaunchConfiguration('world'),
                'pause': LaunchConfiguration('paused'),
                'verbose': LaunchConfiguration('verbose'),
                'server_required': 'true',
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_share, 'launch', 'gzclient.launch.py')),
            condition=IfCondition(LaunchConfiguration('gui')),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            condition=IfCondition(LaunchConfiguration('rviz')),
            output='screen',
        ),
    ])
