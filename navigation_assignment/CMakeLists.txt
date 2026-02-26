cmake_minimum_required(VERSION 3.8)
project(group17_assignment_1)

# Set C++ standard to C++17 if not already set
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

# Find required ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(apriltag_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(action_msgs REQUIRED)

# --- Generate interfaces ---
rosidl_generate_interfaces(${PROJECT_NAME}
        "srv/TableDetection.srv"
        "srv/CorridorDetection.srv"
        "action/FollowCorridor.action"
        DEPENDENCIES geometry_msgs
)

# --- Nav2 Setup Node ---
add_executable(nav2_setup_node src/nav2_setup_node.cpp)
ament_target_dependencies(nav2_setup_node
        rclcpp
        nav2_msgs
        geometry_msgs
        tf2_geometry_msgs
)
install(TARGETS nav2_setup_node DESTINATION lib/${PROJECT_NAME})

# --- Manager Node ---
add_executable(manager_node src/manager_node.cpp)
ament_target_dependencies(manager_node
        rclcpp
        rclcpp_action
        nav2_msgs
        geometry_msgs
        tf2_geometry_msgs
        rosidl_default_runtime
)

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(manager_node "${cpp_typesupport_target}")

install(TARGETS manager_node DESTINATION lib/${PROJECT_NAME})

# --- Table Detector Node ---
add_executable(table_detector_node src/table_detector_node.cpp)
ament_target_dependencies(table_detector_node
        rclcpp
        geometry_msgs
        sensor_msgs
        std_srvs
        tf2_ros
        tf2_geometry_msgs
        rosidl_default_runtime
)
target_link_libraries(table_detector_node "${cpp_typesupport_target}")
install(TARGETS table_detector_node DESTINATION lib/${PROJECT_NAME})

# --- AprilTag Listener Node ---
add_executable(apriltag_listener_node src/apriltag_listener.cpp)
ament_target_dependencies(apriltag_listener_node
        rclcpp
        apriltag_msgs
        tf2_ros
        tf2_geometry_msgs
        geometry_msgs
)
install(TARGETS apriltag_listener_node DESTINATION lib/${PROJECT_NAME})


# --- AprilTag Pose Processor Node ---
add_executable(apriltag_pose_processor_node src/apriltag_pose_processor.cpp)
ament_target_dependencies(apriltag_pose_processor_node
        rclcpp
        geometry_msgs
        tf2_geometry_msgs
        tf2_ros
)

install(TARGETS apriltag_pose_processor_node DESTINATION lib/${PROJECT_NAME})

# --- Corridor Detector Node ---
add_executable(corridor_detector_node src/corridor_detector.cpp)
ament_target_dependencies(corridor_detector_node
        rclcpp
        geometry_msgs
        sensor_msgs
        std_srvs
        rosidl_default_runtime
)

target_link_libraries(corridor_detector_node "${cpp_typesupport_target}")
install(TARGETS corridor_detector_node DESTINATION lib/${PROJECT_NAME})

# --- Corridor Follower Node ---

add_executable(corridor_follower_node src/corridor_follower.cpp)
ament_target_dependencies(corridor_follower_node
        rclcpp
        rclcpp_action
        geometry_msgs
        rosidl_default_runtime
        action_msgs
        sensor_msgs
)
target_link_libraries(corridor_follower_node "${cpp_typesupport_target}")
install(TARGETS corridor_follower_node DESTINATION lib/${PROJECT_NAME})

# --- Install launch directory ---
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY cfg DESTINATION share/${PROJECT_NAME})        # Install the new cfg directory

ament_package()
