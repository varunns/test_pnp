cmake_minimum_required(VERSION 2.8)
project(ControllerPose)
set(CMAKE_CXX_STANDARD 11)
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories(include)
add_executable(pose src/EstimateControllerPose.cpp
					src/main.cpp)
target_link_libraries(pose ${OpenCV_LIBS})