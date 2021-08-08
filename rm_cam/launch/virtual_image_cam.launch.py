import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


# def generate_launch_description():
#     image_path = os.path.join(get_package_share_directory('rm_cam'), 'resource/test.jpg')
#     config = os.path.join(get_package_share_directory('rm_cam'),'config/cam_params.yaml')
#     return LaunchDescription([
#         Node(package='rm_cam',
#             executable='virtual_image_cam',
#             parameters= [ config ],
#             output='screen')
#     ])

def generate_launch_description():
    image_path = os.path.join(get_package_share_directory('rm_cam'), "resource/test.jpg")
    return LaunchDescription([
        Node(package='rm_cam',
            executable='virtual_image_cam',
            parameters=[
                {'image_path': image_path},
                {'camera_name': 'front_camera'},
                {'camera_matrix': [1552.7, 0.0, 640.0, 0.0, 1537.9, 360.0, 0.0, 0.0, 1.0]},
                {'camera_distortion': [0.0, 0.0, 0.0, 0.0, 0.0]},
                {'fps': 30},
            ],
            output='screen')
    ])