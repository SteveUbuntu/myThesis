#! /usr/bin/env python

import rospy
from moveit_python import MoveGroupInterface
from moveit_msgs.msg import MoveItErrorCodes
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

rospy.init_node('moveit_python_tutorial', anonymous=True)

move_group = MoveGroupInterface("arm_group", "g_base")

gripper_frame = 'joint6_flange'

# Setting 2 diffent goals to move constantly from one goal to another until it is terminated
gripper_poses = [Pose(Point(0.0797626, -0.0646115, 0.411097), Quaternion(-0.00300, -0.003, -0.707061, 0.707144)),
                 Pose(Point(0.04560003143886354, -0.05462102695479831,0.31113976263572166), Quaternion(-0.49999999999325395, 0.5000018365991791, -0.49999816339407505, 0.5000000000067462))]

gripper_pose_stamped = PoseStamped()
gripper_pose_stamped.header.frame_id = 'g_base'

while not rospy.is_shutdown():
    for pose in gripper_poses:
        gripper_pose_stamped.header.stamp = rospy.Time.now()

        gripper_pose_stamped.pose = pose

        result = move_group.moveToPose(gripper_pose_stamped, gripper_frame)

        if result:

            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                rospy.loginfo("Trajectory successfully executed!")
            else:
                rospy.logerr("Arm goal in state: %s",
                             move_group.get_move_action().get_state())
        else:
            rospy.logerr("MoveIt failure! No result returned.")

move_group.get_move_action().cancel_all_goals()