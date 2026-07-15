cmake_minimum_required(VERSION 3.8)
project(fire_swarm_environment)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

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
add_executable(hazard_publisher src/hazard_publisher.cpp)

ament_target_dependencies(
  hazard_publisher
  rclcpp
  std_msgs
)

install(
  TARGETS hazard_publisher
  DESTINATION lib/${PROJECT_NAME}
)
add_executable(
  local_hazard_publisher
  src/local_hazard_publisher.cpp
)

ament_target_dependencies(
  local_hazard_publisher
  rclcpp
  std_msgs
)

install(
  TARGETS local_hazard_publisher
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(swarm_goal_dispatcher
  src/swarm_goal_dispatcher.cpp
)

ament_target_dependencies(swarm_goal_dispatcher
  rclcpp
  rclcpp_action
  nav2_msgs
)

install(TARGETS
  swarm_goal_dispatcher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
