cmake_minimum_required(VERSION 2.8.3)
project(rrbot)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
  controller_manager
  joint_state_publisher
  robot_state_publisher
  rospy
  rviz
  std_msgs
  apriltag_ros
)

catkin_package(
  DEPENDS
    roscpp
    gazebo_ros
    controller_manager
    joint_state_publisher
    robot_state_publisher
    rospy
    rviz
    std_msgs
    apriltag_ros
)
## Install ##

foreach(dir config launch meshes urdf worlds)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

install(
  FILES urdf.rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

add_library(plannar_mover_plugin src/plannar_mover.cpp)
target_link_libraries(plannar_mover_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})


# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})


