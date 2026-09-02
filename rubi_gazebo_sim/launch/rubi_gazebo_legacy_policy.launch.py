import glob
import os

from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    sim_share = get_package_share_directory('rubi_gazebo_sim')
    core_share = get_package_share_directory('rubi_control_core')
    gazebo_share = get_package_share_directory('gazebo_ros')
    plugin_prefix = get_package_prefix('rubi_gazebo_plugins')
    system_model_paths = sorted(glob.glob('/usr/share/gazebo-*/models'))
    model_paths = [os.path.join(sim_share, 'models'), *system_model_paths]
    world = os.path.join(sim_share, 'worlds', 'rubi_gazebo_legacy_policy.world')

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
        DeclareLaunchArgument('paused', default_value='false'),
        DeclareLaunchArgument('verbose', default_value='true'),
        DeclareLaunchArgument('namespace', default_value='/rubi_gazebo_legacy'),
        DeclareLaunchArgument('auto_demo', default_value='false'),
        DeclareLaunchArgument('policy_test_ticks', default_value='500'),
        SetEnvironmentVariable(
            'GAZEBO_MODEL_PATH', [os.pathsep.join(model_paths), os.pathsep,
                                  EnvironmentVariable('GAZEBO_MODEL_PATH', default_value='')]),
        SetEnvironmentVariable(
            'GAZEBO_PLUGIN_PATH', [os.path.join(plugin_prefix, 'lib'), os.pathsep,
                                   EnvironmentVariable('GAZEBO_PLUGIN_PATH', default_value='')]),
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        SetEnvironmentVariable(
            'RUBI_GAZEBO_LEGACY_POLICY', LaunchConfiguration('policy')),
        SetEnvironmentVariable(
            'RUBI_GAZEBO_POLICY_VARIANT',
            LaunchConfiguration('controller_variant')),
        SetEnvironmentVariable(
            'RUBI_GAZEBO_TERRAIN_ENCODER',
            LaunchConfiguration('terrain_encoder')),
        SetEnvironmentVariable(
            'RUBI_GAZEBO_TERRAIN_POLICY',
            LaunchConfiguration('terrain_policy')),
        SetEnvironmentVariable(
            'RUBI_GAZEBO_LEGACY_NAMESPACE', LaunchConfiguration('namespace')),
        SetEnvironmentVariable(
            'RUBI_GAZEBO_LEGACY_SELF_TEST', LaunchConfiguration('auto_demo')),
        SetEnvironmentVariable(
            'RUBI_GAZEBO_LEGACY_POLICY_TEST_TICKS',
            LaunchConfiguration('policy_test_ticks')),
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
    ])
