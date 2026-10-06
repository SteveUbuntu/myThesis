#! /usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()    
group = moveit_commander.MoveGroupCommander("arm_group")
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

# Get the current values of the joints
group_variable_values = group.get_current_joint_values()

# Modify the value of joint 5
group_variable_values[5] = -1.5
# Set the new joint value as a target
group.set_joint_value_target(group_variable_values)

# Calculate the plan
plan2 = group.plan()

rospy.sleep(5)

# Execute the trajectory in the simulation
group.go(wait=True)

moveit_commander.roscpp_shutdown()