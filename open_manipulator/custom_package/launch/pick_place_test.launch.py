#!/usr/bin/env python3
#
# Camera (with aligned depth) + object_detector for the pick-and-place pipeline test, one command instead of two terminals:
#   ros2 launch custom_package pick_place_test.launch.py
#   ros2 launch custom_package pick_place_test.launch.py model_path:=/workspace/models/yolo_cubes.pt marker_id:=20
# The detector names tag <marker_id> "marker" (the word used in the test prompt). Start the arm launch and scripted_demo
# separately (they read the keyboard): docs/finetuning_procedure.md, section 6.
#

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def detector(context):
    share = get_package_share_directory('custom_package')
    return [Node(
        package='custom_package', executable='object_detector', name='object_detector', output='screen', emulate_tty=True,
        parameters=[os.path.join(share, 'config', 'object_detector.yaml'), {
            'model_path': LaunchConfiguration('model_path').perform(context),
            'target_tags': [f"{LaunchConfiguration('marker_id').perform(context)}:marker"],
        }],
    )]


def generate_launch_description():
    share = get_package_share_directory('custom_package')
    return LaunchDescription([
        DeclareLaunchArgument('model_path', default_value='/workspace/models/yolo_cubes.pt'),
        DeclareLaunchArgument('marker_id', default_value='20', description='AprilTag id objects are placed on'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'camera_front.launch.py')),
            launch_arguments={'depth': 'true'}.items(),
        ),
        OpaqueFunction(function=detector),
    ])
