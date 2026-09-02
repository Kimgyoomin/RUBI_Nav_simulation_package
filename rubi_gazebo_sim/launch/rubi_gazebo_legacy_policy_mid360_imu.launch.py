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
    core_share = get_package_share_directory('rubi_control_core')
    livox_prefix = get_package_prefix('ros2_livox_simulation')
    livox_share = get_package_share_directory('ros2_livox_simulation')
    gazebo_plugins_prefix = get_package_prefix('gazebo_plugins')

    legacy_launch = os.path.join(
        sim_share, 'launch', 'rubi_gazebo_legacy_policy.launch.py')
    world = os.path.join(
        # sim_share, 'worlds', 'rubi_gazebo_legacy_policy_mid360_imu.world')
        sim_share, 'worlds', 'rubi_navigation_mid360_imu.world')
    xacro_file = os.path.join(
        description_share, 'urdf', 'RUBI_mid360_imu.urdf.xacro')
    rviz_config = os.path.join(
        sim_share, 'config', 'rubi_gazebo_legacy_policy_mid360.rviz')
    robot_description = ParameterValue(
        Command(['xacro', ' ', xacro_file]), value_type=str)
    shutil.copyfile(
        os.path.join(livox_share, 'scan_mode', 'mid360.csv'),
        os.path.join(tempfile.gettempdir(), 'rubi_mid360_imu_mid360.csv'),
    )

    namespace = LaunchConfiguration('namespace')
    torque_off_service = [
        namespace, '/legacy_policy_controller/torque_off']

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=world),
        DeclareLaunchArgument(
            'policy', default_value=os.path.join(
                core_share, 'models', 'rubi_gazebo_legacy_policy.onnx')),
        DeclareLaunchArgument('controller_variant', default_value='legacy'),
        DeclareLaunchArgument(
            'terrain_encoder', default_value=os.path.join(
                core_share, 'models', 'encoder.onnx')),
        DeclareLaunchArgument(
            'terrain_policy', default_value=os.path.join(
                core_share, 'models', 'policy.onnx')),
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('launch_joy_node', default_value='false'),
        DeclareLaunchArgument('joy_device_id', default_value='0'),
        DeclareLaunchArgument('joy_deadzone', default_value='0.05'),
        DeclareLaunchArgument('joy_autorepeat_rate', default_value='20.0'),
        DeclareLaunchArgument('paused', default_value='true'),
        DeclareLaunchArgument('verbose', default_value='true'),
        DeclareLaunchArgument('namespace', default_value='/rubi_gazebo_legacy'),
        DeclareLaunchArgument('auto_demo', default_value='false'),
        DeclareLaunchArgument('policy_test_ticks', default_value='500'),
        DeclareLaunchArgument('safe_start_guard', default_value='true'),
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
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=namespace,
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
            output='screen',
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'device_id': LaunchConfiguration('joy_device_id'),
                'deadzone': LaunchConfiguration('joy_deadzone'),
                'autorepeat_rate': LaunchConfiguration('joy_autorepeat_rate'),
            }],
            condition=IfCondition(LaunchConfiguration('launch_joy_node')),
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
            PythonLaunchDescriptionSource(legacy_launch),
            launch_arguments={
                'world': LaunchConfiguration('world'),
                'policy': LaunchConfiguration('policy'),
                'controller_variant':
                    LaunchConfiguration('controller_variant'),
                'terrain_encoder': LaunchConfiguration('terrain_encoder'),
                'terrain_policy': LaunchConfiguration('terrain_policy'),
                'gui': LaunchConfiguration('gui'),
                'paused': LaunchConfiguration('paused'),
                'verbose': LaunchConfiguration('verbose'),
                'namespace': namespace,
                'auto_demo': LaunchConfiguration('auto_demo'),
                'policy_test_ticks': LaunchConfiguration('policy_test_ticks'),
            }.items(),
        ),
        Node(
            package='rubi_gazebo_sim',
            executable='legacy_torque_off_guard.py',
            arguments=['--service', torque_off_service],
            condition=IfCondition(LaunchConfiguration('safe_start_guard')),
            output='screen',
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
