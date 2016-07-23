cmake_minimum_required(VERSION 2.8.3)
project(an_ros_tools)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
  roscpp
  message_filters
  tf
  geometry_msgs
  nav_msgs
  dji_sdk
)

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

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
#  LIBRARIES an_ros_tools
   CATKIN_DEPENDS roscpp geometry_msgs message_filters nav_msgs tf dji_sdk
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)

add_executable(rel_transform src/rel_transform.cpp)
target_link_libraries(rel_transform ${catkin_LIBRARIES})

add_executable(odom2tf src/odom2tf.cpp)
target_link_libraries(odom2tf ${catkin_LIBRARIES})

add_executable(pos_normalizer src/pos_normalizer.cpp src/transform.cpp)
target_link_libraries(pos_normalizer ${catkin_LIBRARIES})
