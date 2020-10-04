import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='rm_cam',
            executable='task_usb_cam',
            parameters=[
                {'cam_topic_name': 'usb_cam/image_raw'},
                {'usb_cam_path': '/dev/video0'},
                {'cam_width': 1280},
                {'cam_height': 720},
                {'cam_fps': 20}
            ],
            output='screen')
    ])
