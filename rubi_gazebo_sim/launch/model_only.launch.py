from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration
import glob
import os


def generate_launch_description():
    share = get_package_share_directory('rubi_gazebo_sim')
    gazebo_share = get_package_share_directory('gazebo_ros')
    model_path = os.path.join(share, 'models')
    system_model_paths = sorted(glob.glob('/usr/share/gazebo-*/models'))
    all_model_paths = [model_path, *system_model_paths]
    world = os.path.join(share, 'worlds', 'rubi_model_only.world')

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=world),
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('pause', default_value='false'),
        DeclareLaunchArgument('verbose', default_value='true'),
        SetEnvironmentVariable(
            'GAZEBO_MODEL_PATH',
            [
                os.pathsep.join(all_model_paths),
                os.pathsep,
                EnvironmentVariable('GAZEBO_MODEL_PATH', default_value=''),
            ],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_share, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={
                'world': LaunchConfiguration('world'),
                'pause': LaunchConfiguration('pause'),
                'verbose': LaunchConfiguration('verbose'),
                'server_required': 'true',
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_share, 'launch', 'gzclient.launch.py')
            ),
            condition=IfCondition(LaunchConfiguration('gui')),
        ),
    ])
