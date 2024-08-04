# Copyright 2024 Fool Stuck Engineers
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription


def launch_setup(context, *args, **kwargs):
    robot_description_pkg = FindPackageShare(
        [LaunchConfiguration("robot_model"), "_description"]
    ).perform(context)

    load_robot_info = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("wheel_stuck_robot_utils"), "/launch/load_robot_info.launch.py"]
        ),
        launch_arguments={
            "robot_info_param_file": [robot_description_pkg, "/config/robot_info.param.yaml"]
        }.items(),
    )

    return [
        load_robot_info,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            OpaqueFunction(function=launch_setup),
        ]
    )
