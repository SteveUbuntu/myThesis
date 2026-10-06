#! /usr/bin/env python

import sys
import copy
import rospy
# Communicate with the MoveIt Rviz interface
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

# Initiate the moveit_commander module
moveit_commander.roscpp_initialize(sys.argv)
# Initalizing a node
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

# Create an object of RobotCommander() - an interface to our robot
robot = moveit_commander.RobotCommander()
# Create a PlanningSceneInterface object - an interface to the world that surrounds the robot
scene = moveit_commander.PlanningSceneInterface() 
# Create a MoveGroupCommander object - an interface to the manipulator group of joints defined in the setup   
group = moveit_commander.MoveGroupCommander("arm_group")
# Define a topic Publisher - This topic will visualize the planned motion through the MoveIt Rviz interface
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

# Create a pose object - type of msg that will send as a goal.
pose_target = geometry_msgs.msg.Pose()

# Give values to the variables that will define the goal Pose (rosmsg show geometry_msgs/Pose)
pose_target.orientation.x = -0.49999999999325395
pose_target.orientation.y = 0.5000018365991791
pose_target.orientation.z = -0.49999816339407505
pose_target.orientation.w = 0.5000000000067462
pose_target.position.x = 0.04560003143886354
pose_target.position.y = -0.06462102695479831
pose_target.position.z = 0.41113976263572166

# Send the pose
group.set_pose_target(pose_target)
group.set_goal_tolerance(0.01)

# Calculate the plan
plan1 = group.plan()

rospy.sleep(5)

# Execute the trajectory in the simulation
group.go(wait=True)

# Shut down the module
moveit_commander.roscpp_shutdown()