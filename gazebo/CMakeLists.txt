cmake_minimum_required(VERSION 3.0.2)
project(gazebo)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  rospy
  std_msgs
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)


link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})


# For movemodel_plugin plugin
add_library(movepedestrian0_plugin src/movepedestrian0.cc)
target_link_libraries(movepedestrian0_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(movepedestrian1_plugin src/movepedestrian1.cc)
target_link_libraries(movepedestrian1_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(movepedestrian2_plugin src/movepedestrian2.cc)
target_link_libraries(movepedestrian2_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(movepedestrian3_plugin src/movepedestrian3.cc)
target_link_libraries(movepedestrian3_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(movepedestrian4_plugin src/movepedestrian4.cc)
target_link_libraries(movepedestrian4_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

catkin_package(
  DEPENDS 
    roscpp 
    gazebo_ros 
)
