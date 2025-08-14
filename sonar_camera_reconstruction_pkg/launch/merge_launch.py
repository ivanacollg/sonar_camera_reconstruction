import os

from launch import LaunchDescription    
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Parameter file paths
    pkg_path = get_package_share_directory('sonar_camera_reconstruction_pkg')
    monocular_config = os.path.join(pkg_path, 'config', 'monocular_camera_params.yaml')
    sonar_config = os.path.join(pkg_path, 'config', 'sonar_params.yaml')
    merge_config = os.path.join(pkg_path, 'config', 'merge_params.yaml')
    rviz_file = os.path.join(pkg_path, 'rviz', 'merge.rviz')

    return LaunchDescription([
        # merge_node
        Node(
            package='sonar_camera_reconstruction_pkg',
            executable='merge_node.py',
            name='merge_node',
            parameters=[
                monocular_config,
                sonar_config,
                merge_config,
                {
                    'publish_rate': 5,
                    # Subscriber parameters
                    'sonar_topic':'/sonar_oculus_node/M750d/ping',
                    'odom_topic':'/bruce/slam/localization/odom',
                    'img_topic':'/camera/image_raw/compressed',
                    # Publisher parameters
                    'segmented_img_topic':'/sonar_camera_reconstruction/segmented_img/compressed',
                    'merge_cloud_topic':'/sonar_camera_reconstruction/cloud',
                    'feature_image_topic':'/sonar_camera_reconstruction/feature_img/compressed',
                }
            ],
            output='screen',
        ),
        # tf2_ros 
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world2baselink',
            arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
            output='screen',
        ),

        # RViz 
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', rviz_file],
            output='screen',
        ),

    ])