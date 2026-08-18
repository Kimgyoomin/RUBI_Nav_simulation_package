from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    share = get_package_share_directory('rubi_mujoco_sim')
    default_model = os.path.join(share, 'models', 'rubi.xml')

    return LaunchDescription([
        DeclareLaunchArgument('model', default_value=default_model),
        DeclareLaunchArgument('steps', default_value='100'),
        Node(
            package='rubi_mujoco_sim',
            executable='rubi_mujoco_model_smoke',
            arguments=[LaunchConfiguration('model'), LaunchConfiguration('steps')],
            output='screen',
        ),
    ])
