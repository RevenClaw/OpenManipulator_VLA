#!/usr/bin/env python3
#
# Front (main) camera for vla_bridge: Intel RealSense D455 above the table, color only. Both cameras (front + the D435 on
# the wrist): cameras.launch.py, which includes this file.
#   ros2 launch custom_package camera_front.launch.py
#   ros2 launch custom_package camera_front.launch.py serial_no:=_207322250030 color_profile:=640,480,15   # the old, smaller frames
#   ros2 launch custom_package camera_front.launch.py depth:=true     # + depth aligned to color, for object_detector
# Publishes /camera/front/color/image_raw (rgb8, 1280x800 @ 15 Hz: the D455's full color resolution; 15 Hz = the policy's
# control rate; data recorded before 2026-11-22 is 640x480). vla_bridge resizes each frame
# to 224x224 (resize_with_pad) when it builds an observation. Depth, infrared, IMU and pointcloud are off: the policy uses
# only RGB. The D455 needs USB 3 (`lsusb -t`: 5000M).
# depth:=true adds depth aligned to the color image (/camera/front/aligned_depth_to_color/image_raw, 16UC1 mm) for
# object_detector (scripted demos, calibration). Use a USB 3 port for color + depth.
# Colour settings (exposure, white balance, saturation, ...): config/camera_front.yaml (`color_config`).
#

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    color_config = os.path.join(get_package_share_directory('custom_package'), 'config', 'camera_front.yaml')
    rs_launch = os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')
    return LaunchDescription([
        DeclareLaunchArgument('serial_no', default_value='_207322250030',
                              description="RealSense serial with a leading _ (default: the D455 above the table); "
                                          "'' = first device (random with two cameras attached)"),
        DeclareLaunchArgument('color_profile', default_value='1280,800,15', description='width,height,fps of the color stream'),
        DeclareLaunchArgument('color_config', default_value=color_config,
                              description="yaml with the RGB colour settings (exposure, white balance, saturation, ...); '' = defaults"),
        DeclareLaunchArgument('depth', default_value='false', description='true: depth aligned to color (object_detector)'),
        # 848x480: the D455's minimum distance grows with the depth resolution (~52 cm at 1280x720, ~35 cm at 848x480); at
        # 1280x720 the cube tops (~0.49 m from the camera) had no depth at all, only the table and tags around them did.
        DeclareLaunchArgument('depth_profile', default_value='848,480,15',
                              description='width,height,fps of the depth stream (848x480: min distance ~35 cm)'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rs_launch),
            launch_arguments={
                'camera_namespace': 'camera',
                'camera_name': 'front',
                'serial_no': LaunchConfiguration('serial_no'),
                'config_file': LaunchConfiguration('color_config'),
                'enable_color': 'true',
                'rgb_camera.color_profile': LaunchConfiguration('color_profile'),
                'enable_depth': LaunchConfiguration('depth'),
                'depth_module.depth_profile': LaunchConfiguration('depth_profile'),
                'enable_infra1': 'false',
                'enable_infra2': 'false',
                'enable_gyro': 'false',
                'enable_accel': 'false',
                'pointcloud.enable': 'false',
                'align_depth.enable': LaunchConfiguration('depth'),
            }.items(),
        ),
    ])
