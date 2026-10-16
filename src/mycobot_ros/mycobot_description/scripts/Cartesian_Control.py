#!/usr/bin/env python3

import sys
import copy
import geometry_msgs.msg
import moveit_commander
import rospy
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetPositionFK, GetPositionFKRequest
from sensor_msgs.msg import JointState


REFERENCE_FRAME = 'g_base'
END_EFFECTOR_LINK = 'joint6_flange'
TARGET_OFFSET = 0.01


EPSILON = 0.01


def read_firefighter_joint_state():
    rospy.loginfo('Waiting for /firefighter/joint_states...')
    joint_state = rospy.wait_for_message(
        '/firefighter/joint_states',
        JointState,
        timeout=10.0,
    )
    rospy.loginfo('Received joint state from /firefighter/joint_states: %s', joint_state.position)
    return joint_state


def set_start_state_from_joint_state(move_group, joint_state):
    joint_names = move_group.get_joints()
    if not joint_names:
        rospy.logwarn('No joints found for arm_group; using current MoveIt state.')
        move_group.set_start_state_to_current_state()
        return

    positions_by_name = dict(zip(joint_state.name, joint_state.position))
    missing_joint_names = [name for name in joint_names if name not in positions_by_name]
    if missing_joint_names:
        rospy.logwarn('Joint state is missing arm joints %s; using current MoveIt state.', missing_joint_names)
        move_group.set_start_state_to_current_state()
        return

    robot_state = move_group.get_current_state()
    state_position_indices = {
        name: index for index, name in enumerate(robot_state.joint_state.name)
    }
    missing_state_names = [name for name in joint_names if name not in state_position_indices]
    if missing_state_names:
        rospy.logwarn(
            'MoveIt current state is missing arm joints %s; using current MoveIt state.',
            missing_state_names,
        )
        move_group.set_start_state_to_current_state()
        return

    state_positions = list(robot_state.joint_state.position)
    for name in joint_names:
        state_positions[state_position_indices[name]] = positions_by_name[name]
    robot_state.joint_state.position = state_positions

    move_group.set_start_state(robot_state)
    move_group.set_start_state_to_current_state()
    rospy.loginfo('Refreshed MoveIt current state from /firefighter/joint_states.')

def get_pose_from_joint_state(move_group, joint_state):
    joint_names = move_group.get_joints()
    if not joint_names:
        rospy.logwarn('No arm joints available for FK; using current MoveIt pose instead.')
        return move_group.get_current_pose(END_EFFECTOR_LINK).pose

    positions_by_name = dict(zip(joint_state.name, joint_state.position))
    robot_state = move_group.get_current_state()
    state_position_indices = {
        name: index for index, name in enumerate(robot_state.joint_state.name)
    }

    state_positions = list(robot_state.joint_state.position)
    for name in joint_names:
        if name in positions_by_name and name in state_position_indices:
            state_positions[state_position_indices[name]] = positions_by_name[name]

    robot_state.joint_state.position = state_positions

    rospy.wait_for_service('/compute_fk')
    compute_fk = rospy.ServiceProxy('/compute_fk', GetPositionFK)
    request = GetPositionFKRequest()
    request.header.frame_id = REFERENCE_FRAME
    request.fk_link_names = [END_EFFECTOR_LINK]
    request.robot_state = robot_state

    response = compute_fk(request)
    if response.error_code.val != MoveItErrorCodes.SUCCESS:
        rospy.logerr('Forward kinematics failed with MoveIt error code %d.', response.error_code.val)
        return move_group.get_current_pose(END_EFFECTOR_LINK).pose

    if not response.pose_stamped:
        rospy.logerr('Forward kinematics returned no pose for %s.', END_EFFECTOR_LINK)
        return move_group.get_current_pose(END_EFFECTOR_LINK).pose

    return response.pose_stamped[0].pose

def build_target_pose_from_current(current_pose):
    target_pose = copy.deepcopy(current_pose)
    target_pose.position.y -= TARGET_OFFSET
    return target_pose


def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('cartesian_pose_target', anonymous=True)

    try:
        joint_state = read_firefighter_joint_state()
    except rospy.ROSException:
        rospy.logwarn('No /firefighter/joint_states message received; using the current MoveIt state instead.')
        joint_state = None

    move_group = moveit_commander.MoveGroupCommander('arm_group')
    robot = moveit_commander.RobotCommander()
    move_group.set_pose_reference_frame(REFERENCE_FRAME)
    move_group.set_end_effector_link(END_EFFECTOR_LINK)
    move_group.set_planning_time(20.0)
    move_group.allow_replanning(True)
    move_group.set_goal_tolerance(0.01)

    if joint_state is not None:
        set_start_state_from_joint_state(move_group, joint_state)
        current_pose = get_pose_from_joint_state(move_group, joint_state)
    else:
        move_group.set_start_state_to_current_state()
        current_pose = move_group.get_current_pose(END_EFFECTOR_LINK).pose

    target_pose = build_target_pose_from_current(current_pose)

    rospy.loginfo(
        'Current end-effector position in g_base: x=%.3f y=%.3f z=%.3f.',
        current_pose.position.x,
        current_pose.position.y,
        current_pose.position.z,
    )
    rospy.loginfo(
        'Planning Cartesian waypoint to x=%.3f y=%.3f z=%.3f in g_base.',
        target_pose.position.x,
        target_pose.position.y,
        target_pose.position.z,
    )

    waypoints = [copy.deepcopy(current_pose), copy.deepcopy(target_pose)]
    plan, fraction = move_group.compute_cartesian_path(waypoints, EPSILON, avoid_collisions=True)
    rospy.loginfo('Cartesian path completion: %.1f%%', fraction * 100.0)

    if fraction < 1.0:
        rospy.logerr('Cartesian path is incomplete; refusing to execute it.')
        return

    if len(plan.joint_trajectory.points) > 1:
        plan.joint_trajectory.points = plan.joint_trajectory.points[1:]

    refitted_plan = move_group.retime_trajectory(
        robot.get_current_state(),
        plan,
        0.1,
        0.1,
    )
    if not refitted_plan.joint_trajectory.points:
        rospy.logerr('Trajectory time parameterization failed; cannot execute the plan.')
        return
    rospy.loginfo('Trajectory time parameterized at 10% velocity and acceleration.')

    rospy.loginfo('Executing cartesian move.')
    execution_succeeded = move_group.execute(refitted_plan, wait=True)
    if execution_succeeded:
        rospy.loginfo('Motion execution completed.')
    else:
        rospy.logwarn('Motion execution reported failure; the controller did not confirm success.')


if __name__ == '__main__':
    main()
