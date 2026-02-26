import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # 1. Launch Gazebo simulation + RViz
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ir_launch'),
                'launch',
                'assignment_1.launch.py'
            )
        )
    )

    # 2. Launch Setup Nav2 node (localization + navigation initialization and setting initial pose)
    setup_node = Node(
        package='group17_assignment_1',
        executable='nav2_setup_node',
        name='nav2_setup_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 3. Launch Apriltag container (camera + detector)
    apriltag_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                os.path.dirname(__file__),
                'apriltag.launch.py'
            )
        )
    )

    # 4. Launch Apriltag listener node
    listener_node = Node(
        package='group17_assignment_1',
        executable='apriltag_listener_node',
        name='apriltag_listener',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 5. Launch Apriltag processor node
    processor_node = Node(
        package='group17_assignment_1',
        executable='apriltag_pose_processor_node',
        name='apriltag_pose_processor',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 6. Launch Table detector node
    table_detector_node = Node(
        package='group17_assignment_1',
        executable='table_detector_node',
        name='table_detector_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 5. Launch corridor detector and follower nodes
    corridor_detector_node = Node(
        package='group17_assignment_1',
        executable='corridor_detector_node',
        name='corridor_detector',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    corridor_follower_node = Node(
        package='group17_assignment_1',
        executable='corridor_follower_node',
        name='corridor_follower',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )


    # 7. Launch Manager node
    manager_node = Node(
        package='group17_assignment_1',
        executable='manager_node',
        name='manager_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )


    # LaunchDescription
    return LaunchDescription([
        simulation_launch,
        setup_node,
        apriltag_launch,
        listener_node,
        processor_node,
        table_detector_node,
        corridor_detector_node,
        corridor_follower_node,       
        manager_node
    ])
