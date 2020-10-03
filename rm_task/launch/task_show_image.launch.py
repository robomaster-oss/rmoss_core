import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='rm_task',
            node_executable='task_show_image',
            parameters=[
                {'cam_topic_name': 'sim_cam/image_raw'}
            ],
            output='screen')
    ])
