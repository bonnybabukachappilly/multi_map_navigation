cmake_minimum_required(VERSION 3.8)
project(navigations)

include(FindPkgConfig)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# find_package(sqlite3 REQUIRED)
find_package(rosidl_default_generators REQUIRED)


include(FindPkgConfig)
pkg_check_modules(SQLITE3 REQUIRED sqlite3)

include_directories(
  include
  ${SQLITE3_INCLUDE_DIRS}
)

# target_link_libraries(your_node_executable
#   ${SQLITE3_LIBRARIES}
# )

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
  "action/NavigateToGoal.action"
  DEPENDENCIES geometry_msgs std_msgs
)

add_executable(navigate_to_goal_server src/navigate_to_goal_server.cpp)

ament_target_dependencies(navigate_to_goal_server
  rclcpp
  nav2_msgs
  geometry_msgs
  action_msgs
  sqlite3
)

install(TARGETS
  navigate_to_goal_server
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
