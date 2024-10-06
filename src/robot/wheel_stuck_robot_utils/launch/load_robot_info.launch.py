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
