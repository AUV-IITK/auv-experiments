cmake_minimum_required(VERSION 2.8.3)
project(hardware_camera)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roslint
  roscpp
  rospy
  std_msgs
  sensor_msgs
  message_generation
  image_transport
  usb_cam
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
roslint_cpp()

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  #  INCLUDE_DIRS include
  #  LIBRARIES task_buoy
  CATKIN_DEPENDS roscpp rospy std_msgs message_generation
  #  DEPENDS system_lib
)

###########
## Build ##
###########

# set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
# set(CMAKE_CXX_FLAGS "-std=gnu++11 ${CMAKE_CXX_FLAGS}")

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  # include
  # ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
  ${catkin_INCLUDE_DIRS}
  # ${OpenCV_INCLUDE_DIRS}
)
