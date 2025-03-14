import launch
import launch_ros.actions
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    enable_camera_arg = DeclareLaunchArgument(
        "enable_camera", default_value="false", description="Enable Camera"
    )

    enable_camera = LaunchConfiguration("enable_camera")

    camera_node = launch_ros.actions.Node(
        package="camera",
        executable="stream_node",
        name="camera",
        output="screen",
        condition=launch.conditions.IfCondition(enable_camera),
    )

    aim_armor_node = launch_ros.actions.Node(
        package="aim_armor",
        executable="detector_node",
        name="aim_armor",
        output="screen",
    )

    serial_node = launch_ros.actions.Node(
        package="serial",
        executable="serial_node",
        name="serial",
        output="screen",
    )

    return launch.LaunchDescription(
        [enable_camera_arg, camera_node, aim_armor_node, serial_node]
    )
