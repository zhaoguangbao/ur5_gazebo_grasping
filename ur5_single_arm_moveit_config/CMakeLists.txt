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

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
