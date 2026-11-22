#!/usr/bin/env python3
#
# Cameras (front D455 with aligned depth + wrist D435, cameras.launch.py) + object_detector for the pick-and-place pipeline
# test and the demos, one command instead of two terminals:
#   ros2 launch custom_package pick_place_test.launch.py
#   ros2 launch custom_package pick_place_test.launch.py targets:="19:blue marker,25:green marker"     # the defaults
#   ros2 launch custom_package pick_place_test.launch.py wrist:=false     # front camera only
# targets = comma-separated "<AprilTag id>:<name>"; the names are what config/pick_place_test.yaml's color_targets refers to.
# Start the arm launch and scripted_demo separately (they read the keyboard): docs/finetuning_procedure.md, section 6.
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
            'target_tags': [t.strip() for t in LaunchConfiguration('targets').perform(context).split(',') if t.strip()],
        }],
    )]


def generate_launch_description():
    share = get_package_share_directory('custom_package')
    return LaunchDescription([
        DeclareLaunchArgument('model_path', default_value='/workspace/models/yolo_cubes.pt'),
        DeclareLaunchArgument('wrist', default_value='true', description='false: no wrist camera'),
        DeclareLaunchArgument('targets', default_value='19:blue marker,25:green marker',
                              description='comma-separated "<AprilTag id>:<name>" target tags'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'cameras.launch.py')),
            launch_arguments={'depth': 'true', 'wrist': LaunchConfiguration('wrist')}.items(),
        ),
        OpaqueFunction(function=detector),
    ])
