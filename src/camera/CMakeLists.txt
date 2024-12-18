cmake_minimum_required(VERSION 3.8)
project(camera)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


link_directories(${CMAKE_SOURCE_DIR}/dependencies/MVSDK/lib)
include_directories(${CMAKE_SOURCE_DIR}/dependencies/MVSDK/include)



# link_directories(${CMAKE_SOURCE_DIR}/dependencies/HKSDK/lib)
include_directories(${CMAKE_SOURCE_DIR}/dependencies/HKSDK/include)


# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()


add_executable(stream_node "src/stream_node.cpp" "src/mindvision.cpp")
add_executable(video_node "src/video_node.cpp")
# add_executable(test_node "src/test_node.cpp" "src/mindvision.cpp")


target_link_libraries(stream_node MVSDK)
# target_link_libraries(test_node MVSDK)

ament_target_dependencies(
  stream_node
  "rclcpp"
  "sensor_msgs"
  "OpenCV"
  "cv_bridge"
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
# )

target_include_directories(stream_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

# target_include_directories(test_node PUBLIC
# $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
# $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_include_directories(video_node PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

install(TARGETS
  stream_node
  video_node
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