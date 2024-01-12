cmake_minimum_required(VERSION 2.8)
project(test_pnp)
set(CMAKE_CXX_STANDARD 11)
find_package( OpenCV REQUIRED )
find_package( nlohmann_json REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} 
					 ./include)

add_executable(pose src/EstimateControllerPose.cpp
					src/CameraModel.cpp
					main.cpp
					src/P3p.cpp
			   )
target_link_libraries(pose ${OpenCV_LIBS})