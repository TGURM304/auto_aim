cmake_minimum_required(VERSION 3.8)
project(aim_armor)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_STANDARD 20)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(OpenVINO REQUIRED)
find_package(interfaces REQUIRED)
ament_auto_find_build_dependencies()


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


add_executable(detector_node "src/detector_node.cpp" "src/armor_detector.cpp" "src/lights.cpp")

ament_target_dependencies(
  detector_node
  "rclcpp"
  "sensor_msgs"
  "OpenCV"
  "cv_bridge"
  "OpenVINO"
  "interfaces"
)

target_link_libraries(detector_node openvino)

target_include_directories(detector_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address -fsanitize=undefined")
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fsanitize=address -fsanitize=undefined")
# set(CMAKE_LINKER_FLAGS "${CMAKE_LINKER_FLAGS} -fsanitize=address -fsanitize=undefined")


install(TARGETS
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
