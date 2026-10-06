#! /usr/bin/env python

import rospy
from moveit_python import MoveGroupInterface
from moveit_msgs.msg import MoveItErrorCodes

rospy.init_node('moveit_python_tutorial', anonymous=True)

# Create an instance of the MoveGroupInterface interface
move_group = MoveGroupInterface("arm_group", "world")

# Set the name of the joints and their goal position values
joints = ["joint2_to_joint1", "joint3_to_joint2", "joint4_to_joint3",
          "joint5_to_joint4", "joint6_to_joint5", "joint6output_to_joint6"]

pose = [1, 0, 0, 0, 0, 0] # The start pose

while not rospy.is_shutdown():
# Call the moveToJointPosition function passing the names of the joints and their position values, as well as the tolerance of 0.02
    result = move_group.moveToJointPosition(joints, pose, 0.02)
    # Check the output of the function to see if the trajectory has been completed successfully or not
    if result:

        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            rospy.loginfo("Trajectory successfully executed!")
            break
        else:
            rospy.logerr("Arm goal in state: %s",
                         move_group.get_move_action().get_state())
    else:
        rospy.logerr("MoveIt failure! No result returned.")

# Cancel all the goals so the arm stops
move_group.get_move_action().cancel_all_goals()