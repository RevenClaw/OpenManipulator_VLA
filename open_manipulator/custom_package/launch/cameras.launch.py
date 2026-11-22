#!/usr/bin/env python3
#
# Both RealSense cameras, each picked by its serial number (with two cameras attached, "first device" is random):
#   front = D455 above the table   -> /camera/front/color/image_raw (+ aligned depth with depth:=true, for object_detector)
#   wrist = D435 on the gripper    -> /camera/wrist/color/image_raw (color only)
#
#   ros2 launch custom_package cameras.launch.py                   # both, color only (vla_bridge, recording)
#   ros2 launch custom_package cameras.launch.py depth:=true       # + front depth (object_detector, demos, calibration)
#   ros2 launch custom_package cameras.launch.py wrist:=false      # front only
#   ros2 launch custom_package cameras.launch.py front_serial:=_<serial> wrist_serial:=_<serial>   # other cameras
#
# Serials: `rs-enumerate-devices -s`; pass them with a leading _ (otherwise the launch reads them as numbers).
# Front D455: color 1280x800 (full resolution) + depth 848x480 (min distance ~35 cm; 1280x720 sees nothing closer than ~52 cm), wrist D435: color 640x480; all @ 15 Hz (the policy's
# control rate, keep it). The front D455 needs USB 3 (`lsusb -t`: 5000M). The wrist D435 at 640x480 also works on USB 2; a
# larger wrist_profile (e.g. 1280,720,15) needs it on USB 3 too.
#

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

FRONT_SERIAL = '_207322250030'   # D455, mounted above the table
WRIST_SERIAL = '_927522071177'   # D435, on the gripper


def generate_launch_description():
    share = get_package_share_directory('custom_package')
    rs_launch = os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')
    return LaunchDescription([
        DeclareLaunchArgument('front_serial', default_value=FRONT_SERIAL, description='front camera serial, leading _'),
        DeclareLaunchArgument('wrist_serial', default_value=WRIST_SERIAL, description='wrist camera serial, leading _'),
        DeclareLaunchArgument('color_config', default_value=os.path.join(share, 'config', 'camera_front.yaml'),
                              description="front RGB colour settings yaml; '' = camera defaults"),
        DeclareLaunchArgument('depth', default_value='false', description='true: front depth aligned to color'),
        DeclareLaunchArgument('wrist', default_value='true', description='false: front camera only'),
        DeclareLaunchArgument('color_profile', default_value='1280,800,15', description='width,height,fps (front color)'),
        DeclareLaunchArgument('depth_profile', default_value='848,480,15', description='width,height,fps (front depth; 1280x720: nothing closer than ~52 cm)'),
        DeclareLaunchArgument('wrist_profile', default_value='640,480,15', description='width,height,fps (wrist)'),
        # Each camera in its own scoped group: rs_launch.py's arguments must not leak from one camera into the other.
        GroupAction([IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'camera_front.launch.py')),
            launch_arguments={
                'serial_no': LaunchConfiguration('front_serial'),
                'depth': LaunchConfiguration('depth'),
                'color_profile': LaunchConfiguration('color_profile'),
                'depth_profile': LaunchConfiguration('depth_profile'),
                'color_config': LaunchConfiguration('color_config'),
            }.items(),
        )], scoped=True),
        GroupAction([IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rs_launch),
            launch_arguments={
                'camera_namespace': 'camera',
                'camera_name': 'wrist',
                'serial_no': LaunchConfiguration('wrist_serial'),
                'enable_color': 'true',
                'rgb_camera.color_profile': LaunchConfiguration('wrist_profile'),
                'enable_depth': 'false',
                'enable_infra1': 'false',
                'enable_infra2': 'false',
                'enable_gyro': 'false',
                'enable_accel': 'false',
                'pointcloud.enable': 'false',
                'align_depth.enable': 'false',
            }.items(),
        )], scoped=True, condition=IfCondition(LaunchConfiguration('wrist'))),
    ])
