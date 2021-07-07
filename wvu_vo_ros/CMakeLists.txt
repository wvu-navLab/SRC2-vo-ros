cmake_minimum_required(VERSION 3.0)
project(wvu_vo_ros VERSION 1.0.0 LANGUAGES CXX)

SET(CMAKE_CXX_FLAGS "-O3 -std=c++11")

########################
#######DIRECTORY########
########################
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/wvu_vo_ros/bin)
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/wvu_vo_ros/lib)
SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/wvu_vo_ros/lib)

########################
######Dependencies######
########################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_filters
  sensor_msgs
  cv_bridge
  tf
  nav_msgs
  message_generation
)

find_package(OpenCV REQUIRED)

add_service_files(
  FILES
  SetPose.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  nav_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs message_generation
  LIBRARIES ros_stereo_vo
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  wvu_vo/include
)

###########################
#######SUBDIRECTORY########
###########################
# set(BUILD_SHARED_LIBS ON)
add_subdirectory(wvu_vo)

########################
#######LIBRARIES########
########################
add_library(ros_stereo_vo src/ros_stereo_vo.cpp)
target_link_libraries(ros_stereo_vo
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  wvu_vo
)
add_dependencies(ros_stereo_vo wvu_vo_ros_generate_messages_cpp)

##########################
#######EXECUTABLES########
##########################
add_executable(ros_stereo_vo_node src/ros_stereo_vo_node.cpp)
target_link_libraries(ros_stereo_vo_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ros_stereo_vo
)
add_dependencies(ros_stereo_vo_node wvu_vo_ros_generate_messages_cpp)


install(TARGETS ros_stereo_vo_node
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
 install(TARGETS ros_stereo_vo
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
 )

install(DIRECTORY launch/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
   PATTERN ".svn" EXCLUDE)

   install(DIRECTORY config/
       DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
       PATTERN ".svn" EXCLUDE)
