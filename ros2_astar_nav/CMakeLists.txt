cmake_minimum_required(VERSION 3.8)
project(ros2_astar_nav)

# Enable C++17 standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)

# Include directories
include_directories(include)

# List of dependencies
set(DEPENDENCIES
  rclcpp
  rclcpp_action
  rclcpp_components
  nav2_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  tf2_ros
  tf2_geometry_msgs
  tf2
)

# Build shared library for planner and controller
add_library(astar_nav_lib SHARED
  src/astar_planner.cpp
  src/simple_controller.cpp
)
ament_target_dependencies(astar_nav_lib ${DEPENDENCIES})
target_include_directories(astar_nav_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Build executable for navigation node
add_executable(navigation_node src/navigation_node.cpp)
ament_target_dependencies(navigation_node ${DEPENDENCIES})
target_link_libraries(navigation_node astar_nav_lib)

# Install targets
install(TARGETS
  astar_nav_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  navigation_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install include files
install(DIRECTORY include/
  DESTINATION include
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(astar_nav_lib)
ament_export_dependencies(${DEPENDENCIES})

ament_package()
