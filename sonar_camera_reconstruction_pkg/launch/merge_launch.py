import os

from launch import LaunchDescription 
from launch.actions import DeclareLaunchArgument  
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Arguments
    # Declare the argument
    environment_arg = DeclareLaunchArgument(
        'environment',
        default_value='marina_pier',
        description='Environment name (e.g., mrina_pier, marina_seawall, tank)'
    )
    # Get the value of the argument
    environment = LaunchConfiguration('environment')
    env = environment.perform(context)

    # Parameter file paths
    pkg_path = get_package_share_directory('sonar_camera_reconstruction_pkg')
    rviz_file = os.path.join(pkg_path, 'rviz', 'merge.rviz')
    # Use TextSubstitution to properly combine text and LaunchConfiguration
    param_file = os.path.join(pkg_path, 'config',  f'params_{env}.yaml')

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare the argument
    environment_arg = DeclareLaunchArgument(
        'environment',
        default_value='marina_pier',
        description='Environment name (e.g., marina_pier, marina_seawall, tank)'
    )

    environment = LaunchConfiguration('environment')

    # Path setup
    pkg_path = get_package_share_directory('sonar_camera_reconstruction_pkg')
    rviz_file = os.path.join(pkg_path, 'rviz', 'merge.rviz')

    # The function that runs *after* context is available
    def launch_setup(context):
        # Resolve the environment variable properly here
        env = environment.perform(context)

        # Parameter file path depends on environment
        param_file = os.path.join(pkg_path, 'config', f'params_{env}.yaml')

        # If file doesn’t exist, fall back to default
        if not os.path.exists(param_file):
            print(f"[WARN] Param file for '{env}' not found — using default params_marina_pier.yaml")
            param_file = os.path.join(pkg_path, 'config', 'params_marina_pier.yaml')
        print(f"[INFO] Using param file for '{env}'")


        # Nodes
        merge_node = Node(
            package='sonar_camera_reconstruction_pkg',
            executable='merge_node.py',
            name='merge_node',
            parameters=[
                param_file,
                {
                    'publish_rate': 5,
                    'sonar_topic': '/sonar_oculus_node/M750d/ping',
                    'odom_topic': '/bruce/slam/localization/odom',
                    'img_topic': '/camera/image_raw/compressed',
                    'segmented_img_topic': '/sonar_camera_reconstruction/segmented_img/compressed',
                    'merge_cloud_topic': '/sonar_camera_reconstruction/cloud',
                    'feature_image_topic': '/sonar_camera_reconstruction/feature_img/compressed',
                }
            ],
            output='screen',
        )

        tf_node = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world2baselink',
            arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
            output='screen',
        )

        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', rviz_file],
            output='screen',
        )

        return [merge_node, tf_node, rviz_node]

    # Return launch description
    return LaunchDescription([
        environment_arg,
        OpaqueFunction(function=launch_setup),
    ])
