cmake_minimum_required(VERSION 3.8)
project(turtlebot_flocking)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)


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


add_executable(turtlebot_flocking
  src/flocking.cpp
)
target_include_directories(turtlebot_flocking PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
ament_target_dependencies(turtlebot_flocking
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
)

target_link_libraries(turtlebot_flocking
  ${rclcpp_LIBRARIES})

install(TARGETS
  turtlebot_flocking
  DESTINATION ../argos3_ros2_bridge/lib/argos3_ros2_bridge)

ament_package()
