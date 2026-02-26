cmake_minimum_required(VERSION 3.8)
project(group17_assignment_2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(apriltag_msgs REQUIRED)

find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp_action REQUIRED)

find_package(control_msgs REQUIRED)

find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)

# --- Manager Node ---
add_executable(manager_node src/manager_node.cpp)
target_include_directories(manager_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(manager_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(manager_node
        rclcpp
        rclcpp_action
        geometry_msgs
        tf2_geometry_msgs
        moveit_ros_planning_interface
        control_msgs
)

install(TARGETS manager_node DESTINATION lib/${PROJECT_NAME})

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

# --- ColorDetector Node ---
add_executable(color_detector_node src/color_detector_node.cpp)
ament_target_dependencies(color_detector_node
        rclcpp
        cv_bridge
        sensor_msgs
        tf2_ros
        tf2_geometry_msgs
)

target_link_libraries(color_detector_node ${OpenCV_LIBRARIES})

install(TARGETS color_detector_node DESTINATION lib/${PROJECT_NAME})

# --- Install launch directory ---
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY cfg DESTINATION share/${PROJECT_NAME})        # Install the new cfg directory

ament_package()
