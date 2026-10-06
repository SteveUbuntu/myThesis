#!/usr/bin/env python3

import sys

import rospy
import moveit_commander


import math


DEFAULT_JOINTS = [
    30.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0
]


def parse_joint_values(args):
    if len(args) != 6:
        raise ValueError('Expected exactly 6 joint values in degrees.')
    values = [float(v) for v in args]
    return [math.radians(v) for v in values]


def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('analytical_ik_joint_target', anonymous=True)

    move_group = moveit_commander.MoveGroupCommander('arm_group')
    move_group.set_pose_reference_frame('g_base')
    move_group.set_start_state_to_current_state()
    move_group.set_planning_time(20.0)
    move_group.allow_replanning(True)
    move_group.set_goal_tolerance(0.02)

    if len(sys.argv) > 1:
        target_joints = parse_joint_values(sys.argv[1:7])
    else:
        target_joints = parse_joint_values([str(v) for v in DEFAULT_JOINTS])

    current_joints = move_group.get_current_joint_values()
    rospy.loginfo('Current joint values: %s', current_joints)
    rospy.loginfo('Target joint values: %s', target_joints)

    move_group.set_joint_value_target(target_joints)

    plan_result = move_group.plan()
    if isinstance(plan_result, tuple):
        success, plan, _, _ = plan_result
    else:
        success = True
        plan = plan_result

    if success and plan is not None:
        move_group.execute(plan, wait=True)
        rospy.loginfo('Joint-space motion execution completed.')
    else:
        rospy.logerr('Joint-space planning failed!')

    moveit_commander.roscpp_shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as exc:
        rospy.logerr('Error: %s', exc)
        raise
