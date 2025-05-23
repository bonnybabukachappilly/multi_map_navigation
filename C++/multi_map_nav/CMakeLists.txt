cmake_minimum_required(VERSION 3.8)
project(multi_map_nav)

include_directories(
  include
)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(SQLite3 REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp_action REQUIRED)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/NavigateToMap.action"
  DEPENDENCIES geometry_msgs std_msgs
)


add_executable(navigate_to_map_action_server
  src/navigate_to_map_action_server.cpp
  src/navigate_to_map_main.cpp
)

ament_target_dependencies(navigate_to_map_action_server
  rclcpp
  rclcpp_action
  geometry_msgs
  std_msgs
  action_msgs
  nav2_msgs
  SQLite3
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(navigate_to_map_action_server
  ${cpp_typesupport_target}
  sqlite3
)


install(DIRECTORY include/
  DESTINATION include/
)

install(TARGETS
  navigate_to_map_action_server
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  rviz
  maps
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
