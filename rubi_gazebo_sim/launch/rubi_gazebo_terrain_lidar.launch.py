import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    sim_share = get_package_share_directory('rubi_gazebo_sim')
    core_share = get_package_share_directory('rubi_control_core')
    integrated_launch = os.path.join(
        sim_share, 'launch',
        'rubi_gazebo_legacy_policy_mid360_imu.launch.py')

    default_world = os.path.join(
        sim_share,
        'worlds',
        'rubi_navigation_mid360_imu.world')

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=default_world),
        DeclareLaunchArgument(
            'encoder', default_value=os.path.join(
                core_share, 'models', 'encoder.onnx')),
        DeclareLaunchArgument(
            'policy', default_value=os.path.join(
                core_share, 'models', 'policy.onnx')),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('paused', default_value='true'),
        DeclareLaunchArgument('auto_demo', default_value='false'),
        DeclareLaunchArgument('launch_joy_node', default_value='false'),
        DeclareLaunchArgument(
            'namespace', default_value='/rubi_gazebo_legacy'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(integrated_launch),
            launch_arguments={
                'world': LaunchConfiguration('world'),
                
                'controller_variant': 'terrain',
                'terrain_encoder': LaunchConfiguration('encoder'),
                'terrain_policy': LaunchConfiguration('policy'),
                'gui': LaunchConfiguration('gui'),
                'rviz': LaunchConfiguration('rviz'),
                'paused': LaunchConfiguration('paused'),
                'auto_demo': LaunchConfiguration('auto_demo'),
                'launch_joy_node': LaunchConfiguration('launch_joy_node'),
                'namespace': LaunchConfiguration('namespace'),
            }.items(),
        ),
    ])
