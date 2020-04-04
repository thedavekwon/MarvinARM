cmake_minimum_required(VERSION 2.8.3)
project(mimic)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  visualization_msgs
  open_manipulator_msgs
  cmake_modules
  tf
  Eigen3
  open_manipulator_gazebo
)

set(${PROJECT_NAME}_SRCS
  src/mimic.cpp
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs open_manipulator_msgs visualization_msgs open_manipulator_gazebo
  DEPENDS
)

###########
## Build ##
###########

include_directories(
  include
  ${open_manipulator_control_gui_INCLUDE_DIRECTORIES}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(mimic src/mimic.cpp src/Arm.cpp)
target_link_libraries(mimic ${catkin_LIBRARIES})
add_dependencies(mimic mimic)
