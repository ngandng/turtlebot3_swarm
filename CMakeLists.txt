cmake_minimum_required(VERSION 3.8)
project(turtlebot3_swarm)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Generate the compile_commands.json file
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)

include_directories(
  include
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  ${PROJECT_NAME}/turtlebot3_controller.py
  DESTINATION lib/${PROJECT_NAME}/
)

# Instal C++ files
add_executable(move_to_target_node src/main.cpp src/SimpleController.cpp)
ament_target_dependencies(move_to_target_node rclcpp std_msgs tf2 geometry_msgs nav_msgs)

install(TARGETS
  move_to_target_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch meshes urdf worlds models rviz
  DESTINATION share/${PROJECT_NAME}/
)


ament_package()
