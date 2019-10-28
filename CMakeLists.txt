cmake_minimum_required(VERSION 2.8.3)
project(gclib_ros)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  message_runtime
  rospy
  roscpp
  std_msgs
  cmake_modules
  geometry_msgs
  sensor_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(DIRECTORY msg FILES

  Position.msg
  )

add_service_files(DIRECTORY srv FILES

  SingleAxisMotion.srv
  MultiAxisMotion.srv
  SetFloat.srv
  )

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES

  std_msgs
  geometry_msgs
  sensor_msgs
  )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES gdax_interface
   CATKIN_DEPENDS roscpp std_msgs geometry_msgs rospy std_msgs message_runtime message_generation
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  )

add_library(gclib_ros src/main.cpp)

add_dependencies(gclib_ros
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(gclib_ros
  ${catkin_LIBRARIES}
  )
