cmake_minimum_required(VERSION 3.8)
project(cpp_srvcli)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(tutorial_interfaces REQUIRED)

add_executable(server src/add_two_ints_server.cpp)
ament_target_dependencies(server rclcpp example_interfaces)

add_executable(client src/add_two_ints_client.cpp)
ament_target_dependencies(client rclcpp example_interfaces)

add_executable(server_3_add src/add_three_ints_server.cpp)
ament_target_dependencies(server_3_add rclcpp tutorial_interfaces)

add_executable(client_3_add src/add_three_ints_client.cpp)
ament_target_dependencies(client_3_add rclcpp tutorial_interfaces)

install(TARGETS
    server_3_add
    client_3_add
    DESTINATION lib/${PROJECT_NAME})

ament_package()
