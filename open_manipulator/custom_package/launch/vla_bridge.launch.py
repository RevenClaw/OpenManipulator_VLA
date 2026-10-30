#!/usr/bin/env python3
#
# Launches vla_bridge with config/vla_bridge.yaml. Arguments given on the command line override the YAML:
#   ros2 launch custom_package vla_bridge.launch.py server_host:=10.0.0.5 shadow:=true
# Launched this way the bridge has no terminal menu: enable it with
#   ros2 service call /vla_bridge/enable std_srvs/srv/SetBool "{data: true}"
# For the e / d / s / q terminal menu use instead:
#   ros2 run custom_package vla_bridge --ros-args --params-file \
#       $(ros2 pkg prefix custom_package)/share/custom_package/config/vla_bridge.yaml -p shadow:=true
#

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Launch arguments that may override the YAML, with their types. Empty (the default) = keep the YAML value.
OVERRIDES = {
    'server_host': str,
    'server_port': int,
    'prompt': str,
    'shadow': lambda v: v.lower() in ('true', '1', 'yes'),
    'fake_images': lambda v: v.lower() in ('true', '1', 'yes'),
    'max_joint_vel': float,
}


def launch_setup(context):
    params_file = os.path.join(get_package_share_directory('custom_package'), 'config', 'vla_bridge.yaml')
    overrides = {}
    for name, convert in OVERRIDES.items():
        value = LaunchConfiguration(name).perform(context)
        if value != '':
            overrides[name] = convert(value)
    return [
        Node(
            package='custom_package',
            executable='vla_bridge',
            name='vla_bridge',
            output='screen',
            emulate_tty=True,
            parameters=[params_file, overrides],
        )
    ]


def generate_launch_description():
    args = [DeclareLaunchArgument(name, default_value='', description='override vla_bridge.yaml') for name in OVERRIDES]
    return LaunchDescription([*args, OpaqueFunction(function=launch_setup)])
