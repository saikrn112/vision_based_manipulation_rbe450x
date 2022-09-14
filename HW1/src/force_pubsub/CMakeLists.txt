cmake_minimum_required(VERSION 3.8)
project(force_pubsub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tutorial_interfaces REQUIRED)

add_executable(force_server src/force_server.cpp)
ament_target_dependencies(force_server rclcpp geometry_msgs tutorial_interfaces)

add_executable(force_client src/force_client.cpp)
ament_target_dependencies(force_client rclcpp geometry_msgs tutorial_interfaces)

install(TARGETS
    force_server
    force_client
    DESTINATION lib/${PROJECT_NAME})

ament_package()
