from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    description_share = Path(
        get_package_share_directory("rubi_description")
    )
    urdf_path = description_share / "urdf" / "rubi.urdf"
    rviz_path = description_share / "rviz" / "rubi.rviz"
    robot_description = urdf_path.read_text(encoding="utf-8")

    use_gui = LaunchConfiguration("use_gui")
    launch_rviz = LaunchConfiguration("launch_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    common_parameters = {
        "robot_description": robot_description,
        "use_sim_time": use_sim_time,
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_gui",
                default_value="true",
                description="Use joint_state_publisher_gui.",
            ),
            DeclareLaunchArgument(
                "launch_rviz",
                default_value="true",
                description="Launch RViz2 with the RUBI display config.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use a simulator-provided clock.",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[common_parameters],
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                name="joint_state_publisher_gui",
                output="screen",
                condition=IfCondition(use_gui),
                parameters=[common_parameters],
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="joint_state_publisher",
                output="screen",
                condition=UnlessCondition(use_gui),
                parameters=[common_parameters],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", str(rviz_path)],
                condition=IfCondition(launch_rviz),
                parameters=[{"use_sim_time": use_sim_time}],
            ),
        ]
    )
