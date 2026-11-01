#!/usr/bin/env python3
#
# Front (main) camera for vla_bridge: Intel RealSense D435, color only.
#   ros2 launch custom_package camera_front.launch.py
#   ros2 launch custom_package camera_front.launch.py serial_no:=_927522071177 color_profile:=640,480,30
# Publishes /camera/front/color/image_raw (rgb8, 640x480 @ 15 Hz = the policy's control rate). vla_bridge resizes each frame
# to 224x224 (resize_with_pad) when it builds an observation. Depth, infrared, IMU and pointcloud are off: the policy uses
# only RGB, and 640x480 color at 15 Hz also fits a USB 2 link.
#

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    rs_launch = os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')
    return LaunchDescription([
        DeclareLaunchArgument('serial_no', default_value="''",
                              description='RealSense serial (prefix with _, e.g. _927522071177); empty = first device'),
        DeclareLaunchArgument('color_profile', default_value='640,480,15', description='width,height,fps of the color stream'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rs_launch),
            launch_arguments={
                'camera_namespace': 'camera',
                'camera_name': 'front',
                'serial_no': LaunchConfiguration('serial_no'),
                'enable_color': 'true',
                'rgb_camera.color_profile': LaunchConfiguration('color_profile'),
                'enable_depth': 'false',
                'enable_infra1': 'false',
                'enable_infra2': 'false',
                'enable_gyro': 'false',
                'enable_accel': 'false',
                'pointcloud.enable': 'false',
                'align_depth.enable': 'false',
            }.items(),
        ),
    ])
