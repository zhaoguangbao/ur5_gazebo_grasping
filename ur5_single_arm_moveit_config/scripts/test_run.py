#! /usr/bin/env python2.7
import sys
import rospy
import moveit_commander

moveit_commander.roscpp_initializer.roscpp_initialize(sys.argv)
rospy.init_node('move_group_grasp', anonymous=True)
robot = moveit_commander.robot.RobotCommander()

arm_group = moveit_commander.move_group.MoveGroupCommander("arm")
hand_group = moveit_commander.move_group.MoveGroupCommander("gripper")

#hand_group.set_named_target("close")
#plan = hand_group.go()


arm_group.set_named_target("home")
plan = arm_group.go()
print("Home")
rospy.sleep(3)

# joint_values = arm_group.get_current_joint_values()
#
# joint_values[0] = joint_values[0]-1.57
# arm_group.set_joint_value_target(joint_values)
#
# arm_group.go(wait=True)
# print("Point 3")

arm_group.set_named_target("home_j")
plan = arm_group.go()
print("Point 1")
rospy.sleep(3)

# target_pose = arm_group.get_current_pose().pose
# target_pose.position.x = target_pose.position.x + 0.1
# arm_group.set_pose_target(target_pose)
# arm_group.go()

target_pose = arm_group.get_current_pose().pose
target_pose.position.z = target_pose.position.z + 0.1
arm_group.set_pose_target(target_pose)
arm_group.go()
print('end')

rospy.sleep(5)
moveit_commander.roscpp_initializer.roscpp_shutdown()