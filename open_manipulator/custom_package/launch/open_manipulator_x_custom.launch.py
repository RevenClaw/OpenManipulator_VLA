#!/usr/bin/env python3
#
# Copyright 2026 ROBOTIS CO., LTD.
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
#

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    custom_pkg_dir = get_package_share_directory('custom_package')
    default_config_path = os.path.join(
        custom_pkg_dir, 'config', 'configured_home_position.yaml'
    )

    bringup_launch_dir = get_package_share_directory('open_manipulator_bringup')

    return LaunchDescription([
        DeclareLaunchArgument(
            'init_position_file',
            default_value=default_config_path,
            description='Path to recorded initial position YAML configuration file',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    bringup_launch_dir, 'launch', 'open_manipulator_x.launch.py'
                )
            ),
            launch_arguments={
                'init_position_file': LaunchConfiguration('init_position_file')
            }.items(),
        ),
    ])
