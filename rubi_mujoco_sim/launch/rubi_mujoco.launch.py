import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim_share = get_package_share_directory('rubi_mujoco_sim')
    core_share = get_package_share_directory('rubi_control_core')

    arguments = [
        DeclareLaunchArgument(
            'model', default_value=os.path.join(sim_share, 'models', 'rubi.xml')),
        DeclareLaunchArgument(
            'encoder', default_value=os.path.join(core_share, 'models', 'encoder.onnx')),
        DeclareLaunchArgument(
            'policy', default_value=os.path.join(core_share, 'models', 'policy.onnx')),
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('real_time_factor', default_value='1.0'),
        DeclareLaunchArgument('run_as_fast_as_possible', default_value='false'),
        DeclareLaunchArgument('namespace', default_value='rubi_mujoco'),
        DeclareLaunchArgument('command_timeout', default_value='0.5'),
        DeclareLaunchArgument('auto_demo', default_value='false'),
        DeclareLaunchArgument('screenshot_path', default_value=''),
    ]

    node = Node(
        package='rubi_mujoco_sim',
        executable='rubi_mujoco_node',
        namespace=LaunchConfiguration('namespace'),
        name='controller',
        output='screen',
        parameters=[{
            'model': LaunchConfiguration('model'),
            'encoder': LaunchConfiguration('encoder'),
            'policy': LaunchConfiguration('policy'),
            'gui': LaunchConfiguration('gui'),
            'real_time_factor': LaunchConfiguration('real_time_factor'),
            'run_as_fast_as_possible': LaunchConfiguration('run_as_fast_as_possible'),
            'command_timeout': LaunchConfiguration('command_timeout'),
            'auto_demo': LaunchConfiguration('auto_demo'),
            'screenshot_path': LaunchConfiguration('screenshot_path'),
        }],
    )
    return LaunchDescription(arguments + [node])
