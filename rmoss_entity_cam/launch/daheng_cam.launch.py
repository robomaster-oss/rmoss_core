'''
Author: holakk
Date: 2021-11-06 19:47:28
LastEditors: holakk
LastEditTime: 2021-11-07 22:22:01
Description: file content
'''
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    param_path = os.path.join(get_package_share_directory("rmoss_entity_cam"), "config/cam_param.yaml")

    daheng_cam_node = Node(
        package="rmoss_entity_cam",
        executable="daheng_cam",
        name="daheng_camera",
        parameters=[param_path]
    )
    ld = LaunchDescription()
    ld.add_action(daheng_cam_node)

    return ld