#!/usr/bin/env python2.7
import sys
import rospy
import moveit_commander
import geometry_msgs
import tf

 
moveit_commander.roscpp_initializer.roscpp_initialize(sys.argv)
rospy.init_node('move_group_grasp', anonymous=True)
robot = moveit_commander.robot.RobotCommander()

arm_group = moveit_commander.move_group.MoveGroupCommander("arm")
hand_group = moveit_commander.move_group.MoveGroupCommander("gripper")

#hand_group.set_named_target("close")
#plan = hand_group.go()

# hand_group.set_named_target("close")
gripper_pose = hand_group.get_current_joint_values()
gripper_pose[0] = 0.35
hand_group.set_joint_value_target(gripper_pose)
plan = hand_group.go()
print("Point 3")

arm_group.set_named_target("home")
plan = arm_group.go()

print("Point 1")

# Open
#hand_group.set_joint_value_target([9.800441184282249e-05, -9.800441184282249e-05, 9.800441184282249e-05, 9.800441184282249e-05, -9.800441184282249e-05, 9.800441184282249e-05])
#hand_group.go(wait=True)
#print("Point 2")

hand_group.set_named_target("open")
plan = hand_group.go()
print("Point 2")

# 1.0 joint space plan
pose_target = arm_group.get_current_pose().pose
# joint_values = arm_group.get_current_joint_values()
#
# joint_values[0] = joint_values[0]-1
# arm_group.set_joint_value_target(joint_values)
#
# arm_group.go(wait=True)
# print("Point 3")
#
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
