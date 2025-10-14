cmake_minimum_required(VERSION 3.8)
project(armor_pipeline)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE debug)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(armor_detect REQUIRED)

file(GLOB_RECURSE
  ARM_FINDER_SRC_FILES
  CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
)

set(FINDER_SRC ${CMAKE_CURRENT_SOURCE_DIR}/src/pipeline)
set(NODE_SRC ${CMAKE_CURRENT_SOURCE_DIR}/src/node)

add_executable(frame_capture
  ${FINDER_SRC}/frame_capture.cpp
  ${NODE_SRC}/frame_cap_node.cpp
)
target_include_directories(frame_capture PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
ament_target_dependencies(frame_capture
  "rclcpp"
  "cv_bridge"
  "sensor_msgs"
)

add_executable(armor_finder
  ${FINDER_SRC}/armor_finder.cpp
  ${NODE_SRC}/armor_find_node.cpp
)
target_include_directories(armor_finder PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
ament_target_dependencies(armor_finder
  "rclcpp"
  "cv_bridge"
  "sensor_msgs"
  "armor_detect"
)

install(TARGETS
  frame_capture
  armor_finder
  DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
)

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


