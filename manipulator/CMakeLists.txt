cmake_minimum_required(VERSION 2.8.3)
project(moveit_manipulation)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)

# Eigen 3.2 (Wily) only provides EIGEN3_INCLUDE_DIR, not EIGEN3_INCLUDE_DIRS
if(NOT EIGEN3_INCLUDE_DIRS)
    set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs sensor_msgs geometry_msgs
        interactive_markers moveit_core moveit_visual_tools moveit_ros_planning moveit_ros_planning_interface
        pluginlib geometric_shapes tf2_ros tf2_eigen tf2_geometry_msgs)

find_package(PkgConfig)

catkin_package(
        CATKIN_DEPENDS
        std_msgs
        sensor_msgs
        roscpp
        geometry_msgs
        moveit_core
        moveit_visual_tools
        moveit_ros_planning_interface
        interactive_markers
        tf2_ros tf2_eigen tf2_geometry_msgs
        DEPENDS
        EIGEN3
)

include_directories (
        SYSTEM
        ${EIGEN3_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)
link_directories(
        ${catkin_LIBRARY_DIRS}
)

#libraries
add_library(Manipulator src/Manipulator.cpp)
target_link_libraries(Manipulator ${catkin_LIBRARIES})

#application
add_executable(moveit_manipulation src/main.cpp)
target_link_libraries(moveit_manipulation Manipulator ${catkin_LIBRARIES})
