cmake_minimum_required(VERSION 3.8)
project(aim_armor)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 20)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


link_directories(${CMAKE_SOURCE_DIR}/dependencies/onnxruntime/lib)
include_directories(${CMAKE_SOURCE_DIR}/dependencies/onnxruntime/include)


# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()


add_executable(test1 "src/test/test1.cpp")
add_executable(detector_node "src/detector_node.cpp")
# add_executable(openvinotest "src/test/openvinotest.cpp")


# ament_target_dependencies(
#   openvinotest
#   "rclcpp"
#   "OpenCV"
#   "openvino"
# )

ament_target_dependencies(
  test1
  "rclcpp"
  "sensor_msgs"
  "OpenCV"
  "cv_bridge"
)

ament_target_dependencies(
  detector_node
  "rclcpp"
  "sensor_msgs"
  "OpenCV"
  "cv_bridge"
)

# target_include_directories(openvinotest PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_include_directories(test1 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

  target_include_directories(detector_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

install(TARGETS 
  # openvinotest
  test1
  detector_node
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
