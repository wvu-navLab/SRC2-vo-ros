cmake_minimum_required(VERSION 3.0)
project(wvu_vo VERSION 1.0.0 LANGUAGES CXX)

SET(CMAKE_CXX_FLAGS "-O3 -std=c++11")

########################
#######DIRECTORY########
########################
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/wvu_vo/bin)
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/wvu_vo/lib)
SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/wvu_vo/lib)

########################
######Dependencies######
########################
find_package(OpenCV REQUIRED)

INCLUDE_DIRECTORIES(
	include
	src
	${PROJECT_SOURCE_DIR}
)

#######################
########Library########
#######################
add_library(wvu_vo
	src/opencv_utils.cpp
	src/CameraInfo.cpp
	src/Image.cpp
	src/Tracker.cpp
	src/StereoImage.cpp
	src/StereoVO.cpp)

target_link_libraries(wvu_vo 
	${OpenCV_LIBS}
 )

########################
########Examples########
########################
add_executable(example_kitti examples/example_kitti.cpp)
target_link_libraries(example_kitti 
	wvu_vo
 )

add_executable(example_src2 examples/example_src2.cpp)
target_link_libraries(example_src2 
	wvu_vo
 )

#########################
##########Tests##########
#########################
#TODO