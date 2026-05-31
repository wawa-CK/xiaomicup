cmake_minimum_required(VERSION 3.8)
project(wild_glint_hunt)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(protocol REQUIRED)
find_package(cyberdog_msg QUIET)
find_package(gazebo_msgs QUIET)
find_package(gazebo_dev REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_library(LCM_LIBRARY lcm REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/VisionBall.msg"
  "msg/VisionBallArray.msg"
  DEPENDENCIES std_msgs
)

include_directories(include)

add_library(robot_interface_impl
  src/robot_interface_sim.cpp
  src/robot_interface_real.cpp
)
ament_target_dependencies(robot_interface_impl rclcpp geometry_msgs nav_msgs sensor_msgs)
ament_target_dependencies(robot_interface_impl protocol)
if(cyberdog_msg_FOUND)
  target_compile_definitions(robot_interface_impl PUBLIC WGH_HAS_CYBERDOG_MSG)
  ament_target_dependencies(robot_interface_impl cyberdog_msg)
endif()
if(gazebo_msgs_FOUND)
  target_compile_definitions(robot_interface_impl PUBLIC WGH_HAS_GAZEBO_MSGS)
  ament_target_dependencies(robot_interface_impl gazebo_msgs)
endif()
target_link_libraries(robot_interface_impl ${LCM_LIBRARY})

add_executable(vision_node src/vision_node.cpp)
ament_target_dependencies(vision_node rclcpp std_msgs sensor_msgs cv_bridge image_transport OpenCV)
rosidl_target_interfaces(vision_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(vision_node ${OpenCV_LIBRARIES})

add_executable(reset_robot_pose src/reset_robot_pose.cpp)
ament_target_dependencies(reset_robot_pose rclcpp)

add_executable(simulated_sensors_node src/simulated_sensors_node.cpp)
ament_target_dependencies(simulated_sensors_node rclcpp geometry_msgs nav_msgs sensor_msgs tf2_ros OpenCV)
ament_target_dependencies(simulated_sensors_node protocol)
ament_target_dependencies(simulated_sensors_node gazebo_dev)
if(gazebo_msgs_FOUND)
  ament_target_dependencies(simulated_sensors_node gazebo_msgs)
endif()
target_link_libraries(simulated_sensors_node ${OpenCV_LIBRARIES})

add_executable(check_camera_topic src/check_camera_topic.cpp)
ament_target_dependencies(check_camera_topic rclcpp sensor_msgs)

add_executable(path_planner_node src/path_planner_node.cpp)
ament_target_dependencies(path_planner_node rclcpp std_msgs geometry_msgs nav_msgs)
ament_target_dependencies(path_planner_node protocol)
rosidl_target_interfaces(path_planner_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(path_planner_node robot_interface_impl)

add_executable(state_machine_node src/state_machine_node.cpp)
ament_target_dependencies(state_machine_node rclcpp std_msgs geometry_msgs nav_msgs sensor_msgs)
ament_target_dependencies(state_machine_node protocol)
rosidl_target_interfaces(state_machine_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(state_machine_node robot_interface_impl)

install(TARGETS vision_node simulated_sensors_node check_camera_topic reset_robot_pose path_planner_node state_machine_node robot_interface_impl
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config msg include DESTINATION share/${PROJECT_NAME})
ament_package()
