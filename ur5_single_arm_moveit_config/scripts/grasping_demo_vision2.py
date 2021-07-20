#! /usr/bin/env python2.7
import sys
import copy
import rospy
import moveit_commander
import geometry_msgs
import tf

from moveit_commander import MoveGroupCommander

import moveit_msgs.msg
from geometry_msgs.msg import Pose
from copy import deepcopy


def set_gripper_pos(pos):
    gripper_pose = hand_group.get_current_joint_values()
    # index from ur5.srdf
    gripper_pose[2] = pos
    hand_group.set_joint_value_target(gripper_pose)
    hand_group.go()


def callback(pose):
    object_position_info = pose.position
    object_orientation_info = pose.orientation

    # moveit_commander.roscpp_initialize(sys.argv)
    # rospy.init_node('move_group_grasp', anonymous=True)
    # robot = moveit_commander.robot.RobotCommander()

    # arm_group = moveit_commander.move_group.MoveGroupCommander("arm")
    # hand_group = moveit_commander.move_group.MoveGroupCommander("gripper")

    arm_group.set_named_target("home_j")
    plan = arm_group.go()

    print("Point 1")

    # Open
    # hand_group.set_joint_value_target([9.800441184282249e-05, -9.800441184282249e-05,   9.800441184282249e-05, 9.800441184282249e-05, -9.800441184282249e-05, 9.800441184282249e-05])
    # hand_group.go(wait=True)
    # print("Point 2")
    hand_group.set_named_target("open")
    plan = hand_group.go()
    print("Point 2")

    pose_target = arm_group.get_current_pose().pose

    # Block point top
    pose_target.position.x = object_position_info.x
    pose_target.position.y = object_position_info.y
    pose_target.position.z = object_position_info.z + 0.25

    arm_group.set_pose_target(pose_target)
    arm_group.go(wait=True)
    print("Point 3")

    # Block point1
    # pose_target.position.z = pose_target.position.z - 0.07
    pose_target.position.z = pose_target.position.z - 0.12

    arm_group.set_pose_target(pose_target)
    arm_group.go(wait=True)
    print("Point 4")
    rospy.sleep(3)

    # hand_group.set_named_target("close")
    # plan = hand_group.go()
    set_gripper_pos(0.4)
    print("Point 5")
    rospy.sleep(2)

    # pose_target.position.x = pose_target.position.x - 0.1
    pose_target.position.z = pose_target.position.z + 0.25
    arm_group.set_pose_target(pose_target)
    plan = arm_group.go()
    print("Point 6")

    # pose_target.position.x = pose_target.position.x + 0.5
    # arm_group.go()
    # rospy.sleep(3)
    # print("Point 7")

    hand_group.set_named_target("open")
    plan = hand_group.go()
    print("Point 8")

    arm_group.set_named_target('home_j')
    arm_group.go()

    # moveit_commander.roscpp_shutdown()


if __name__ == "__main__":
    rospy.init_node('object_position_sub_And_grasp_node', anonymous=True)
    moveit_commander.roscpp_initialize(sys.argv)
    arm_group = moveit_commander.move_group.MoveGroupCommander("arm")
    hand_group = moveit_commander.move_group.MoveGroupCommander("gripper")
    arm_group.set_named_target('home_j')
    arm_group.go()
    rospy.sleep(1)
    rate = rospy.Rate(1.0)
    # Avoid the detected object pose error caused by the robot block the object
    rospy.Subscriber("/objection_position_pose", Pose, callback, queue_size=1)
    while not rospy.is_shutdown():
        rospy.spin()
        rate.sleep()
    moveit_commander.roscpp_shutdown()

    # rate = rospy.Rate(10.0)
    # while not rospy.is_shutdown():
    #     try:
    #         print("11")
    #         rospy.spin()
    #     except KeyboardInterrupt:
    #         print("Shutting down")
    #
    #     rate.sleep()
