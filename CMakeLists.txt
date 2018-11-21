cmake_minimum_required(VERSION 2.8.3)
project(pheeno_ros)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

#### Uncomment these two to use pi cam with C++ ####
# find_package(raspicam REQUIRED)
# find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  nav_msgs
  sensor_msgs
  #### Uncomment these two to use pi cam with C++ ####
  # cv_brdige
  # image_transport
)


catkin_package(
  #### Uncomment these four to use pi cam with C++ ####
  #  INCLUDE_DIRS include
  #  LIBRARIES raspicam
  #  CATKIN_DEPENDS actionlib actionlib_msgs geometry_msgs roscpp rospy std_msgs image_transport cv_bridge nav_msgs sensor_msgs
  #  DEPENDS system_lib OpenCV
)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  #### Uncomment these two to use pi cam with C++ ####
  # ${OpenCV_INCLUDE_DIRS}
  # ${raspicam_INCLUDE_DIRS}
)


#### Uncomment this section to use pi cam with C++ ####
# INCLUDE(CheckCXXCompilerFlag)
# CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
# CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
# IF(COMPILER_SUPPORTS_CXX11)
#     SET(CMAKE_CXX_FLAGS "-std=c++11")
# ELSEIF(COMPILER_SUPPORTS_CXX0X)
#     SET(CMAKE_CXX_FLAGS "-std=c++0x")
# ELSE()
#     MESSAGE(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
# ENDIF()


#### Uncomment this section to use pi cam with C++ ####
# target_link_libraries(raspicam_node
#   ${catkin_LIBRARIES}
#   ${OpenCV_LIBRARIES}
#   ${raspicam_CV_LIBS}
#)

#### Uncomment this section to use pi cam with C++ ####
# add_executable(raspicam_node
#   src/raspicam_ros_pub.cpp)



###########
# Install #
###########

add_executable(obstacle_avoidance src/command_line_parser.cpp src/pheeno_robot.cpp src/obstacle_avoidance.cpp)
target_link_libraries(obstacle_avoidance ${catkin_LIBRARIES})

add_executable(random_walk src/command_line_parser.cpp src/pheeno_robot.cpp src/random_walk.cpp)
target_link_libraries(random_walk ${catkin_LIBRARIES})

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/pheeno_robot.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
## (Uncomment raspicam_node to build it)
install(TARGETS obstacle_avoidance random_walk # raspicam_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
