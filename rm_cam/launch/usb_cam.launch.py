import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('rm_cam'),'config/cam_params.yaml')
    return LaunchDescription([
        Node(package='rm_cam',
            executable='usb_cam',
            parameters=[ config ],
            output='screen')
    ])
