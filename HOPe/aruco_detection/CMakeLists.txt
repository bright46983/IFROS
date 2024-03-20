cmake_minimum_required(VERSION 3.1)
project(aruco_detection)

set (CMAKE_CXX_STANDARD 11)
find_package(OpenCV REQUIRED)

include_directories(${OPENCV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/include)

link_directories(${OpenCV_LIBRARY_DIRS})

set(aruco_detection_src
    src/main.cpp
   )
add_executable(aruco_detection ${aruco_detection_src})
target_link_libraries(aruco_detection
    ${OpenCV_LIBRARIES}
    )

target_compile_options(aruco_detection
    PRIVATE -O3 -std=c++11
    )


