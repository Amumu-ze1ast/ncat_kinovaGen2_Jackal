#!/usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


## First initialize `moveit_commander`_ and a `rospy`_ node:
moveit_commander.roscpp_initialize(sys.argv)

# Initialize ROS node
rospy.init_node('moveit_straight_line_movement', anonymous=True)

## This interface can be used to plan and execute motions:
group_name_arm = "manipulator"
group_name_gripper = "gripper"

move_group_arm = moveit_commander.MoveGroupCommander(group_name_arm)
move_group_gripper = moveit_commander.MoveGroupCommander(group_name_gripper)

## Planning to a Joint Goal
joint_goal_arm = move_group_arm.get_current_joint_values()
joint_goal_gripper = move_group_gripper.get_current_joint_values()


robot = moveit_commander.RobotCommander()  

degree_arm = []
degree_gripper = []

degree_arm1 = 90
degree_arm2 = 180
degree_arm3 = 90
degree_arm4 = 0
degree_arm5 = 70
degree_arm6 = 0

degree_gripper1 = 70 # 0-70 for all grippers, 0-86 = for separate grippers
degree_gripper2 = 70
degree_gripper3 = 70

degree_arm.insert(0, degree_arm1)
degree_arm.insert(1, degree_arm2) 
degree_arm.insert(2, degree_arm3)
degree_arm.insert(3, degree_arm4)
degree_arm.insert(4, degree_arm5)
degree_arm.insert(5, degree_arm6)
degree_gripper.insert(0, degree_gripper1)
degree_gripper.insert(1, degree_gripper2)
degree_gripper.insert(2, degree_gripper3)

joint_goal_arm[0] = degree_arm[0] * pi/180
joint_goal_arm[1] = degree_arm[1] * pi/180
joint_goal_arm[2] = degree_arm[2] * pi/180
joint_goal_arm[3] = degree_arm[3] * pi/180
joint_goal_arm[4] = degree_arm[4] * pi/180
joint_goal_arm[5] = degree_arm[5] * pi/180

#joint_goal_gripper[0] = degree_gripper[0] * pi/180
#joint_goal_gripper[1] = degree_gripper[1] * pi/180
#joint_goal_gripper[2] = degree_gripper[2] * pi/180

plan = move_group_arm.go(joint_goal_arm, wait=True)


# Display trajectory
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

## any AttachedCollisionObjects and add our plan to the trajectory.
display_trajectory = moveit_msgs.msg.DisplayTrajectory()
display_trajectory.trajectory_start = robot.get_current_state()
display_trajectory.trajectory.append(plan)

# Publish the plan
display_trajectory_publisher.publish(display_trajectory);

#print ("============ Press `Enter` to execute a saved path ...")
#input()

# The go command can be called with joint values, poses, or without any parameters
move_group_arm.go(joint_goal_arm, wait=True)
move_group_gripper.go(joint_goal_gripper, wait=True)

# Calling ``stop()`` ensures that there is no residual movement
move_group_arm.stop()
move_group_gripper.stop()
