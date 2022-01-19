import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    param_path = os.path.join(get_package_share_directory("rmoss_entity_cam"), "config/cam_param.yaml")

    mv_cam_node = Node(
        package="rmoss_entity_cam",
        executable="mindvision_cam",
        name="mindvision_camera",
        parameters=[param_path],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(mv_cam_node)

    return ld