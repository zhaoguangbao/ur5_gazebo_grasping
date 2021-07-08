#!/usr/bin/env python2.7
import sys

import numpy as np
import rospy
import moveit_commander
import geometry_msgs
import tf

deg2rad = np.pi/180

joint_states_values = np.array([
    [42, 267, 82, -81, 271, -47],
    [11, -97, 104, -98, -88, -78],
    [11, -96, 109, -103, -88, -77]
])*deg2rad


def set_gripper_pos(pos):
    gripper_pose = hand_group.get_current_joint_values()
    # index from ur5.srdf
    gripper_pose[2] = pos
    hand_group.set_joint_value_target(gripper_pose)
    hand_group.go()


def test():
    # 1.0 go home
    arm_group.set_named_target("home")
    plan = arm_group.go()
    print("Go home")

    # 2.0 open gripper
    set_gripper_pos(0.0)
    print("Open gripper")

    # 3.0 close gripper
    set_gripper_pos(0.8)
    print("Close gripper")

    # 4.0
    hand_group.set_named_target("open")
    plan = hand_group.go()
    print("Open gripper")

    # joint space
    joint_values = arm_group.get_current_joint_values()
    # rad
    joint_values[0] = joint_values[0]-1
    arm_group.set_joint_value_target(joint_values)
    arm_group.go(wait=True)
    arm_group.set_named_target("home")
    plan = arm_group.go()
    print("Joint space 1")


moveit_commander.roscpp_initializer.roscpp_initialize(sys.argv)
rospy.init_node('move_group_grasp', anonymous=True)
robot = moveit_commander.robot.RobotCommander()

arm_group = moveit_commander.move_group.MoveGroupCommander("arm")
# joints = arm_group.get_joints()
# print(joints)
hand_group = moveit_commander.move_group.MoveGroupCommander("gripper")
# joints = hand_group.get_joints()
# print(joints)

# # ABORTED: No motion plan found. No execution attempted
# # hand_group.set_named_target("close")
# hand_group.set_named_target("grasp")
# plan = hand_group.go()
# print("Grasp")

# test()

arm_group.set_named_target('home')
arm_group.go()
print('home')

# joint_values = np.array([-174, -73, -100, -98, 88, -175])*deg2rad
row, _ = joint_states_values.shape
for i in range(row):


    joint_values = joint_states_values[i, :]
    arm_group.set_joint_value_target(joint_values)
    arm_group.go()
    print('Joint plan', i)
    rospy.sleep(duration=1.0)
# joint_values = np.array([12, 263, 109, -103, 272, -77])*deg2rad
# arm_group.set_joint_value_target(joint_values)
# arm_group.go()
# print('Joint plan')

# pose_target = arm_group.get_current_pose().pose
# pose_target.position.x = 0.4
# pose_target.position.y = -0.2
# pose_target.position.z = 0.775
# pose_target.orientation.x = 0.0
# pose_target.orientation.y = 0.707
# pose_target.orientation.z = 0.0
# pose_target.orientation.w = 0.707
# # arm_group.set_pose_target(pose_target)
# # arm_group.go()
# # print('Pos target')
#
# ori_target = [0.0, 0.707, 0.0, 0.707]
# arm_group.set_orientation_target(ori_target)
# arm_group.go()
# print('Pos target')

# pose_target = arm_group.get_current_pose().pose
# print(pose_target)

# Block point
# 2.0 able
# pose_target.position.x = 0.5
# pose_target.position.y = 0.05
# pose_target.position.z = 0.7

# arm_group.set_pose_target(pose_target)
# arm_group.go(wait=True)
# print("Point 3")

# Block point
# pose_target.position.x = 0.4
# pose_target.position.y = 0.0
# pose_target.position.z = pose_target.position.z-0.07
#
#
#
# arm_group.set_pose_target(pose_target)
# arm_group.go(wait=True)
# print("Point 4")
#
#
# hand_group.set_named_target("close")
# plan = hand_group.go()
# print("Point 5")
#
# pose_target.position.z = pose_target.position.z+0.05
# arm_group.set_pose_target(pose_target)
# plan = arm_group.go()
# print("Point 6")
#
#
# pose_target.position.z = pose_target.position.z+0.05
# pose_target.position.x = 0.5
# arm_group.set_pose_target(pose_target)
# plan = arm_group.go()
# print("Point 7")
#
#
#
# hand_group.set_named_target("open")
# plan = hand_group.go()
# print("Point 8")
rospy.sleep(5)
moveit_commander.roscpp_initializer.roscpp_shutdown()
