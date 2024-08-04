import yaml
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParameter

from launch import LaunchDescription


def launch_setup(context, *args, **kwargs):
    param_file = LaunchConfiguration("robot_info_param_file").perform(context)
    with open(param_file, "r") as f:
        robot_info = yaml.safe_load(f)["/**"]["ros__parameters"]
    return [SetParameter(name=n, value=v) for (n, v) in robot_info.items()]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_info_param_file"),
            OpaqueFunction(function=launch_setup),
        ]
    )
