cmake_minimum_required(VERSION 3.8)
project(camera)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


set(MVSDK_DIR "${CMAKE_SOURCE_DIR}/dependencies/MVSDK")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(MVSDK REQUIRED)

include_directories("${CMAKE_SOURCE_DIR}/include")

add_executable(stream_node "src/stream_node.cpp")
add_executable(video_node "src/video_node.cpp")
# add_executable(test_node "src/test_node.cpp")
# add_executable(gray_node "src/gray_node.cpp")

ament_target_dependencies(
  stream_node
  "rclcpp"
  "sensor_msgs"
  "OpenCV"
  "cv_bridge"
#   "MVSDK"
)

target_link_libraries(stream_node
  MVSDK
)

ament_target_dependencies(
  video_node
  "rclcpp"
  "sensor_msgs"
  "OpenCV"
  "cv_bridge"
)

# ament_target_dependencies(
#   test_node
#   "rclcpp"
#   "OpenCV"
# #   "MVSDK"
# )

# target_link_libraries(test_node
#   MVSDK
# )

# ament_target_dependencies(
#   gray_node
#   "rclcpp"
#   "sensor_msgs"
#   "OpenCV"
#   "cv_bridge"
# #   "MVSDK"
# )

# target_link_libraries(gray_node
#   MVSDK
# )


install(TARGETS
  stream_node
  video_node
  # gray_node
  # test_node
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
