cmake_minimum_required(VERSION 3.8)
project(closest_point_finder)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(rclcpp REQUIRED)
find_package(motion_utils REQUIRED)
find_package(nav_msgs REQUIRED)

# Include directories for header files
include_directories(include)

# Executable target for the closest_point_finder node
add_executable(closest_point_finder src/closest_point_finder.cpp)

# Link dependencies
ament_target_dependencies(closest_point_finder rclcpp motion_utils nav_msgs)

# Installation settings
install(TARGETS
  closest_point_finder
  DESTINATION lib/${PROJECT_NAME})

ament_package()
