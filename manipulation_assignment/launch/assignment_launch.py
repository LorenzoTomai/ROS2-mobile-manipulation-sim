import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
def generate_launch_description():

    # 1. Launch Gazebo simulation + RViz
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ir_launch'),
                'launch',
                'assignment_2.launch.py'
            )
        )
    )

    # 2. Launch Apriltag container (camera + detector)
    apriltag_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                os.path.dirname(__file__),
                'apriltag.launch.py'
            )
        )
    )

    # 3. Launch Apriltag listener node
    listener_node = Node(
        package='group17_assignment_2',
        executable='apriltag_listener_node',
        name='apriltag_listener',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    moveit_config_pkg = "ir_movit_config"
    moveit_config = MoveItConfigsBuilder(
        "ir_gripper",
        package_name=moveit_config_pkg
    ).to_moveit_configs()

    # 4. Launch Manager node
    manager_node = Node(
        package='group17_assignment_2',
        executable='manager_node',
        name='manager_node',
        output='screen',
        parameters=[{'use_sim_time': True},moveit_config.to_dict(),]
    )

    # 5. Launch Color Detector node
    color_detector_node = Node(
        package='group17_assignment_2',
        executable='color_detector_node',
        name='color_detector_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # LaunchDescription
    return LaunchDescription([
        simulation_launch,
        apriltag_launch,
        listener_node,
        manager_node,
        color_detector_node
    ])
