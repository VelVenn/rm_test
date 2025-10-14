cmake_minimum_required(VERSION 3.8)
project(armor_detect)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE debug)

find_package(OpenCV REQUIRED)
find_package(ament_cmake REQUIRED)

file(GLOB_RECURSE
  ARM_DET_SRC_FILES
  CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
)

add_library(armor_detect STATIC ${ARM_DET_SRC_FILES})
target_include_directories(armor_detect PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/>
)
ament_target_dependencies(armor_detect "OpenCV")

install(
  DIRECTORY include/
  DESTINATION include/
)

install(TARGETS armor_detect
  EXPORT ArmorDetect
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(ArmorDetect HAS_LIBRARY_TARGET)
ament_export_dependencies(OpenCV)

ament_package()

