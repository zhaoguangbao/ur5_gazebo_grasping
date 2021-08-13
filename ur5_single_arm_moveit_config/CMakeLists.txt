cmake_minimum_required(VERSION 3.1.3)
project(ur5_single_arm_moveit_config)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        tf
        gpd_ros
        # move group package
        moveit_core
        moveit_ros_perception
        moveit_ros_move_group
        moveit_ros_planning_interface
        # interactive_markers
        )
find_package(PCL 1.9 REQUIRED)
include_directories($PCL_INCLUDE_DIRS)


find_library(GPD_LIB NAMES gpd PATHS /usr/local/lib PATH_SUFFIXES lib NO_DEFAULT_PATH)
if (GPD_LIB)
    message(STATUS "Library GPD found in ${GPD_LIB}")
else()
    message(FATAL_ERROR "Library GPD not found")
endif()
include_directories(${GPD_LIB_INCLUDE_DIR})
message(STATUS "gpd_include_dir: ${GPD_LIB_INCLUDE_DIR}")

catkin_package(CATKIN_DEPENDS roscpp gpd_ros tf)


include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS})

add_executable(test_movegroup src/${PROJECT_NAME}/test_movegroup.cpp)
# add_dependencies(test_movegroup)
target_link_libraries(test_movegroup ${catkin_LIBRARIES})

add_executable(test_tf_grasp src/${PROJECT_NAME}/test_tf_grasp.cpp)
add_dependencies(test_tf_grasp gpd_ros_gencpp) # gpd_ros/GraspConfig.h: No such file or directory
target_link_libraries(test_tf_grasp ${catkin_LIBRARIES})

add_executable(test_pcd2rviz src/${PROJECT_NAME}/test_pcd2rviz.cpp)
target_link_libraries(test_pcd2rviz ${catkin_LIBRARIES})

add_executable(test_savepcdfrompc2 src/${PROJECT_NAME}/test_savepcdfrompc2.cpp)
target_link_libraries(test_savepcdfrompc2 ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(test_sampleaboveplane src/${PROJECT_NAME}/test_sampleaboveplane.cpp)
target_link_libraries(test_sampleaboveplane ${catkin_LIBRARIES} ${GPD_LIB})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
