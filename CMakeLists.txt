cmake_minimum_required(VERSION 3.0)
project(AprilOptical)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
link_directories(/usr/lib/x86_64-linux-gnu)
link_directories(/usr/local/lib/)

include_directories(/usr/local/include/apriltag/)

include_directories(/usr/local/include/opencv2)
include_directories(-L/usr/local/lib)


#find_package(OpenCV)


find_package(OpenCV REQUIRED )

set(SOURCE_FILES main.cpp features_tracking.cpp tag_detection_features.cpp)
add_executable(AprilOptical ${SOURCE_FILES})

target_link_libraries(AprilOptical pthread apriltag opencv_highgui  ${OpenCV_LIBS})
