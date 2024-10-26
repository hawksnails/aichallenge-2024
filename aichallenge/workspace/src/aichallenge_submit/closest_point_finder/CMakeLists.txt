cmake_minimum_required(VERSION 3.8)
project(closest_point_finder)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_executable(closest_point_finder
  src/closest_point_finder.cpp
)

ament_package()
