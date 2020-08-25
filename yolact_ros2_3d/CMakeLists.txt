cmake_minimum_required(VERSION 3.5)
project(yolact_ros2_3d)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(gb_visual_detection_3d_msgs REQUIRED)
find_package(yolact_ros2_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  lifecycle_msgs
  sensor_msgs
  gb_visual_detection_3d_msgs
  yolact_ros2_msgs
  cv_bridge
)

include_directories(
  include/
  ${OpenCV_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
  src/yolact_ros2_3d/YolactROS23D.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(yolact_ros2_3d_node src/yolact3d_node.cpp)
ament_target_dependencies(yolact_ros2_3d_node ${dependencies})
target_link_libraries(yolact_ros2_3d_node ${PROJECT_NAME})

target_include_directories(yolact_ros2_3d_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

#########
#INSTALL#
#########

install(DIRECTORY include/
  DESTINATION include/
)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME}/)

install(TARGETS
  ${PROJECT_NAME}
  yolact_ros2_3d_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_libraries(${PROJECT_NAME})

ament_package()
