from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    device_arg = DeclareLaunchArgument(
        'device',
        default_value='0',
        description='Device ID for the camera'
    )


    apriltag_yaml = os.path.join(
        get_package_share_directory('group17_assignment_2'),    # modified package name
        'cfg',
        'tags_36h11.yaml'
    )


    apriltag_container = ComposableNodeContainer(
        name='apriltag_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        arguments=['--ros-args', '--log-level', 'error'],     # to avoid unnecessary warnings on the terminal

        composable_node_descriptions=[

            ComposableNode(
                package='realsense2_camera',
                plugin='realsense2_camera::RealSenseNodeFactory',
                name='camera',
                namespace='camera'
            ),

            ComposableNode(
                package='image_proc',
                plugin='image_proc::RectifyNode',
                name='rectify',
                namespace='camera',
                remappings=[
                    ('image', '/rgb_camera/image_raw'),
                    ('camera_info', '/rgb_camera/camera_info')
                ],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),

            ComposableNode(
                package='apriltag_ros',
                plugin='AprilTagNode',
                name='apriltag',
                namespace='apriltag',
                remappings=[
                    ('/apriltag/image_rect', '/rgb_camera/image'),
                    ('/camera/camera_info', '/rgb_camera/camera_info')
                ],
                parameters=[apriltag_yaml],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
        ],
        output='screen'
    )

    return LaunchDescription([
        device_arg,
        apriltag_container
    ])
