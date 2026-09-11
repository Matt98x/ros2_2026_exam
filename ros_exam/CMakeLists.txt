cmake_minimum_required(VERSION 3.8)
project(ros_exam)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ClosestObstacle.msg"
  "srv/GetAverageVelocity.srv"
  DEPENDENCIES std_msgs
)

add_executable(robot_monitor src/robot_monitor.cpp)
target_compile_features(robot_monitor PUBLIC cxx_std_17)
target_compile_options(robot_monitor PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(robot_monitor rclcpp rcl_interfaces geometry_msgs sensor_msgs std_msgs)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(robot_monitor "${cpp_typesupport_target}")
install(TARGETS robot_monitor DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/safe_drive.py scripts/average_velocity.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
ament_export_dependencies(rosidl_default_runtime)
ament_package()
