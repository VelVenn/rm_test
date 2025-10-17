cmake_minimum_required(VERSION 3.10)

project(cv_related)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_BUILD_TYPE Debug)

find_package(OpenCV REQUIRED)

add_executable(armor armor.cpp)

add_executable(color_matcher color_matcher.cpp)
target_link_libraries(color_matcher PRIVATE ${OpenCV_LIBS})

add_executable(mouse_event mouse_event.cpp)
target_link_libraries(mouse_event PRIVATE ${OpenCV_LIBS})

add_executable(recorder recorder.cpp)
target_link_libraries(recorder PRIVATE ${OpenCV_LIBS})

add_executable(apple_detect apple_detect.cpp)
target_link_libraries(apple_detect PRIVATE ${OpenCV_LIBS})

add_executable(defog defog.cpp)
target_link_libraries(defog PRIVATE ${OpenCV_LIBS})

add_executable(calib calibration.cpp)
target_link_libraries(calib PRIVATE ${OpenCV_LIBS})