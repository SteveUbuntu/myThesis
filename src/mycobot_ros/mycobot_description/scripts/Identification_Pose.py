#!/usr/bin/env python3

import sys

import geometry_msgs.msg
import moveit_commander
import rospy
from sensor_msgs.msg import JointState


REFERENCE_FRAME = 'g_base'
END_EFFECTOR_LINK = 'joint6_flange'


def set_start_state_from_joint_state(move_group, joint_state):
    robot_state = move_group.get_current_state()
    positions_by_name = dict(zip(joint_state.name, joint_state.position))
    state_position_indices = {
        name: index for index, name in enumerate(robot_state.joint_state.name)
    }
    joint_names = move_group.get_active_joints()
    missing_names = [
        name for name in joint_names
        if name not in positions_by_name or name not in state_position_indices
    ]
    if missing_names:
        rospy.logwarn('Joint state is missing arm joints %s; using current MoveIt state.', missing_names)
        move_group.set_start_state_to_current_state()
        return

    positions = list(robot_state.joint_state.position)
    for name in joint_names:
        positions[state_position_indices[name]] = positions_by_name[name]
    robot_state.joint_state.position = positions
    move_group.set_start_state(robot_state)


def execute_pose_target(move_group, target_pose, description):
    move_group.set_pose_target(target_pose, end_effector_link=END_EFFECTOR_LINK)
    plan_result = move_group.plan()
    if isinstance(plan_result, tuple):
        success, plan, _, _ = plan_result
    else:
        plan = plan_result
        success = plan is not None and bool(plan.joint_trajectory.points)

    if not success or plan is None or not plan.joint_trajectory.points:
        rospy.logerr('Planning failed for %s.', description)
        move_group.clear_pose_targets()
        return False

    execution_succeeded = move_group.execute(plan, wait=True)
    move_group.stop()
    move_group.clear_pose_targets()
    if execution_succeeded:
        rospy.loginfo('%s completed.', description)
    else:
        rospy.logwarn('%s execution reported failure; continuing after the execution attempt.', description)
    return True


def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_interface_camera', anonymous=True)

    move_group = moveit_commander.MoveGroupCommander('arm_group')
    move_group.set_pose_reference_frame(REFERENCE_FRAME)
    move_group.set_end_effector_link(END_EFFECTOR_LINK)
    move_group.set_planning_time(20.0)
    move_group.allow_replanning(True)
    move_group.set_goal_tolerance(0.01)

    try:
        joint_state = rospy.wait_for_message(
            '/firefighter/joint_states',
            JointState,
            timeout=10.0,
        )
        set_start_state_from_joint_state(move_group, joint_state)
    except rospy.ROSException:
        rospy.logwarn('No /firefighter/joint_states message received; using current MoveIt state.')
        move_group.set_start_state_to_current_state()

    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = 0.169999
    target_pose.position.y = 0.020000
    target_pose.position.z = 0.26213
    target_pose.orientation.x = 0.707122
    target_pose.orientation.y = -0.707065
    target_pose.orientation.z = -0.000411
    target_pose.orientation.w = 0.006100

    rospy.loginfo(
        'Moving %s to target pose in %s: x=%.6f y=%.6f z=%.6f.',
        END_EFFECTOR_LINK,
        REFERENCE_FRAME,
        target_pose.position.x,
        target_pose.position.y,
        target_pose.position.z,
    )
    move_group.set_pose_target(target_pose, end_effector_link=END_EFFECTOR_LINK)
    plan_result = move_group.plan()
    if isinstance(plan_result, tuple):
        success, plan, _, _ = plan_result
    else:
        plan = plan_result
        success = plan is not None and bool(plan.joint_trajectory.points)

    if not success or plan is None or not plan.joint_trajectory.points:
        rospy.logerr('Planning failed for the camera target pose.')
        move_group.clear_pose_targets()
        moveit_commander.roscpp_shutdown()
        return

    execution_succeeded = move_group.execute(plan, wait=True)
    move_group.stop()
    move_group.clear_pose_targets()
    if execution_succeeded:
        rospy.loginfo('Camera target pose reached.')
    else:
        rospy.logwarn('Execution reported failure. The controller did not confirm the camera target pose.')

    moveit_commander.roscpp_shutdown()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass