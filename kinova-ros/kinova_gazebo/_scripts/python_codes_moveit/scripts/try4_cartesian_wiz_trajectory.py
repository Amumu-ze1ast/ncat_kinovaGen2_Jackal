#!/usr/bin/env python3

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
import copy

rospy.init_node('move_group_interface_tutorial', anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "arm"
move_group = moveit_commander.MoveGroupCommander(group_name)

move_group.set_max_velocity_scaling_factor(0.1)

display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)

waypoints = []

current_pose = move_group.get_current_pose().pose
waypoints.append(copy.deepcopy(current_pose))

target_pose = geometry_msgs.msg.Pose()
target_pose.orientation.w = 1.0
target_pose.position.x = current_pose.position.x
target_pose.position.y = current_pose.position.y + 0.2
target_pose.position.z = current_pose.position.z
waypoints.append(copy.deepcopy(target_pose))

target_pose.position.x = current_pose.position.x - 0.2
waypoints.append(copy.deepcopy(target_pose))

target_pose.position.y = current_pose.position.y - 0.2
waypoints.append(copy.deepcopy(target_pose))

target_pose.position.x = current_pose.position.x + 0.2
waypoints.append(copy.deepcopy(target_pose))

(plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)

for point in plan.joint_trajectory.points:
    point.time_from_start *= 10  # Scale the time by 10x to slow down

display_trajectory = moveit_msgs.msg.DisplayTrajectory()
display_trajectory.trajectory_start = robot.get_current_state()
display_trajectory.trajectory.append(plan)
display_trajectory_publisher.publish(display_trajectory)

move_group.execute(plan)

rospy.spin()
