cmake_minimum_required(VERSION 3.8)
project(trajectory_handler)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(autoware_auto_planning_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# サービスファイルを指定
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/PathInfo.srv"
)


# ヘッダーの生成を確認するためのエクスポート
ament_export_dependencies(rosidl_default_runtime)

include_directories(${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(trajectory_publisher src/trajectory_publisher.cpp)
ament_target_dependencies(trajectory_publisher rclcpp autoware_auto_planning_msgs)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(trajectory_publisher ${cpp_typesupport_target})

add_executable(path_setter src/path_setter.cpp)
ament_target_dependencies(path_setter rclcpp autoware_auto_planning_msgs)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(path_setter ${cpp_typesupport_target})

add_executable(trajectory_handler_node src/trajectory_handler.cpp)
ament_target_dependencies(trajectory_handler_node rclcpp autoware_auto_planning_msgs)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(trajectory_handler_node ${cpp_typesupport_target})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


install(TARGETS
  trajectory_publisher
  path_setter
  trajectory_handler_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
