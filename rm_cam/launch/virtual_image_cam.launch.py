import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    image_path = os.path.join(get_package_share_directory('rm_cam'), "res/test.jpg")
    return LaunchDescription([
        Node(package='rm_cam',
            executable='virtual_image_cam',
            parameters=[
                {'cam_topic_name': 'virtual_cam/image_raw'},
                {'image_path': image_path},
                {'cam_fps': 30}
            ],
            output='screen')
    ])
