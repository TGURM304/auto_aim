cmake_minimum_required(VERSION 3.8)
project(test)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)


# 获取 ROS 发行版环境变量
if(DEFINED ENV{ROS_DISTRO})
  set(ROS_DISTRO $ENV{ROS_DISTRO})
else()
  message(FATAL_ERROR "ROS_DISTRO is not defined")
endif()

# 根据 ROS 版本设置宏
if(ROS_DISTRO STREQUAL "humble")
  add_definitions(-DROS_HUMBLE)
elseif (ROS_DISTRO STREQUAL "iron")
  add_definitions(-DROS_IRON)
else()
  message(WARNING "Unknown ROS distribution: ${ROS_DISTRO}")
endif()


add_executable(getstream src/getstream.cpp)
add_executable(record src/record.cpp)

ament_target_dependencies(
  getstream
  "rclcpp"
  "OpenCV"
  "sensor_msgs"
  "cv_bridge"
)

ament_target_dependencies(
  record
  "rclcpp"
  "OpenCV"
  "sensor_msgs"
  "cv_bridge"
)

install(TARGETS getstream
  DESTINATION lib/${PROJECT_NAME})


install(TARGETS record
DESTINATION lib/${PROJECT_NAME})


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

ament_package()
