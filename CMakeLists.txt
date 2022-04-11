cmake_minimum_required(VERSION 3.0.2)
project(april_land)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  tf
  geometry_msgs
  mav_msgs
  nav_msgs
  trajectory_msgs
  roscpp
  apriltag_ros
  mav_trajectory_generation
  eigen_conversions
)

find_package(Eigen3 REQUIRED)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  nav_msgs
  mav_msgs
  trajectory_msgs
)


###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES april_land
#  CATKIN_DEPENDS geometry_msgs roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(
  pose_estimator_node
  src/pose_estimator_node.cpp)
add_executable(
  trajectory_generation_node
  src/trajectory_generation_node.cpp)
add_executable(
  trajectory_test_node
  src/traj_test.cpp)


#############
## Install ##
#############

target_link_libraries(
  pose_estimator_node
  ${catkin_LIBRARIES}
)

target_link_libraries(
  trajectory_generation_node
  ${catkin_LIBRARIES}
)

target_link_libraries(
  trajectory_test_node
  ${catkin_LIBRARIES}
)

install(
  TARGETS
  pose_estimator_node
)

install(
  TARGETS
  trajectory_generation_node
)

install(
  TARGETS
  trajectory_test_node
)

install(
  DIRECTORY config data doc launch rviz scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  add_default_clang_tooling(
    DISABLE_CLANG_FORMAT
  )
endif(cmake_clang_tools_FOUND)
