#!/usr/bin/env python3

import sys

import rospy
import moveit_commander
import geometry_msgs.msg
from sensor_msgs.msg import JointState
from gazebo_ros_link_attacher.srv import Attach, AttachRequest


def read_firefighter_joint_state():
    rospy.loginfo('Waiting for /firefighter/joint_states...')
    joint_state = rospy.wait_for_message('/firefighter/joint_states', JointState, timeout=10.0)
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
    state_position_indices = {name: index for index, name in enumerate(robot_state.joint_state.name)}
    missing_state_names = [name for name in joint_names if name not in state_position_indices]
    if missing_state_names:
        rospy.logwarn('MoveIt current state is missing arm joints %s; using current MoveIt state.', missing_state_names)
        move_group.set_start_state_to_current_state()
        return

    state_positions = list(robot_state.joint_state.position)
    for name in joint_names:
        state_positions[state_position_indices[name]] = positions_by_name[name]
    robot_state.joint_state.position = state_positions

    move_group.set_start_state(robot_state)
    rospy.loginfo('Set MoveIt start state from /firefighter/joint_states.')


def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_interface_python', anonymous=True)

    try:
        joint_state = read_firefighter_joint_state()
    except rospy.ROSException:
        rospy.logwarn('No /firefighter/joint_states message received; using the current MoveIt state instead.')
        joint_state = None

    move_group = moveit_commander.MoveGroupCommander('arm_group')
    move_group.set_pose_reference_frame('g_base')
    move_group.set_planning_time(20.0)
    move_group.allow_replanning(True)
    move_group.set_goal_tolerance(0.01)

    if joint_state is not None:
        set_start_state_from_joint_state(move_group, joint_state)
    else:
        move_group.set_start_state_to_current_state()

    pick_pose = geometry_msgs.msg.Pose()
    pick_pose.position.x = 0.169999
    pick_pose.position.y = -0.050000
    pick_pose.position.z = 0.26213
    pick_pose.orientation.x = 0.707122
    pick_pose.orientation.y = -0.707065
    pick_pose.orientation.z = -0.000411
    pick_pose.orientation.w = 0.006100

    rospy.loginfo('Target pose: x=%.6f y=%.6f z=%.6f',
                  pick_pose.position.x,
                  pick_pose.position.y,
                  pick_pose.position.z)

    rospy.loginfo('Target orientation: x=%.6f y=%.6f z=%.6f w=%.6f',
                  pick_pose.orientation.x,
                  pick_pose.orientation.y,
                  pick_pose.orientation.z,
                  pick_pose.orientation.w)

    rospy.loginfo('Moving to target pose.')
    move_group.set_pose_target(pick_pose, end_effector_link='joint6_flange')

    plan_result = move_group.plan()
    if isinstance(plan_result, tuple):
        success, plan, _, _ = plan_result
    else:
        success = True
        plan = plan_result

    if success and plan is not None:
        execution_succeeded = move_group.execute(plan, wait=True)
        if execution_succeeded:
            rospy.loginfo('Motion execution completed.')
        else:
            rospy.logwarn('Motion execution reported failure; continuing with gripper close after execution ended.')
        
        pick_pose.position.z = 0.23213

        rospy.loginfo('Moving closer to execute Pick Action.')
        move_group.set_start_state_to_current_state()
        move_group.set_pose_target(pick_pose, end_effector_link='joint6_flange')

        pick_plan_result = move_group.plan()
        if isinstance(pick_plan_result, tuple):
            pick_success, pick_plan, _, _ = pick_plan_result
        else:
            pick_success = True
            pick_plan = pick_plan_result

        if pick_success and pick_plan is not None:
            pick_execution_succeeded = move_group.execute(pick_plan, wait=True)
            if pick_execution_succeeded:
                rospy.loginfo('Pick pose completed.')
            else:
                rospy.logwarn('Pick pose motion reported failure; continuing after execution ended.')
        else:
            rospy.logwarn('Pick pose planning failed; continuing to gripper close.')

        gripper_group = moveit_commander.MoveGroupCommander('gripper')
        gripper_target = 'close'
        gripper_group.set_named_target(gripper_target)
        rospy.loginfo('Closing gripper.')
        gripper_execution_succeeded = gripper_group.go(wait=True)
        if gripper_execution_succeeded:
            rospy.loginfo('Gripper motion to the %s state completed.', gripper_target)
        else:
            rospy.logwarn('Gripper motion reported failure; continuing after execution ended.')

        try:
            attach_service_name = '/link_attacher_node/attach'
            rospy.wait_for_service(attach_service_name, timeout=10.0)
            attach_service = rospy.ServiceProxy(attach_service_name, Attach)
            attach_request = AttachRequest(
                model_name_1='firefighter',
                link_name_1='joint6_flange',
                model_name_2='cube_clone',
                link_name_2='link_1',
            )
            attach_response = attach_service(attach_request)
            if attach_response.ok:
                rospy.loginfo('Attached firefighter to cube_clone.')
            else:
                rospy.logerr('The link attacher service rejected the attachment.')
        except (rospy.ROSException, rospy.ServiceException) as error:
            rospy.logerr('Failed to attach links: %s', error)

        rospy.loginfo('Moving to the init_pose.')
        move_group.set_start_state_to_current_state()
        move_group.set_named_target('init_pose')

        init_plan_result = move_group.plan()
        if isinstance(init_plan_result, tuple):
            init_success, init_plan, _, _ = init_plan_result
        else:
            init_success = False
            init_plan = None

        if init_success and init_plan is not None:
            init_execution_succeeded = move_group.execute(init_plan, wait=True)
            if init_execution_succeeded:
                rospy.loginfo('Init_pose motion achieved.')
            else:
                rospy.logwarn('Init_pose motion reported failure.')

            place_pose = geometry_msgs.msg.Pose()
            place_pose.position.x = 0.063285
            place_pose.position.y = 0.072739
            place_pose.position.z = 0.17213
            place_pose.orientation.x = -0.717632
            place_pose.orientation.y = -0.011673
            place_pose.orientation.z = 0.012253
            place_pose.orientation.w = 0.696217

            rospy.loginfo('Moving to target_pose_2.')
            move_group.set_start_state_to_current_state()
            move_group.set_pose_target(place_pose, end_effector_link='joint6_flange')

            place_plan_result = move_group.plan()
            if isinstance(place_plan_result, tuple):
                place_success, place_plan, _, _ = place_plan_result
            else:
                place_success = True
                place_plan = place_plan_result
            
            if place_success and place_plan is not None:
                place_execution_succeeded = move_group.execute(place_plan, wait=True)
                if place_execution_succeeded:
                    rospy.loginfo('Motion execution_2 completed.')
                else:
                    rospy.logwarn('place_motion reported failure.')

                place_pose.position.y = 0.092739
                
                rospy.loginfo('Moving closer to execute Place Action.')
                move_group.set_start_state_to_current_state()
                move_group.set_pose_target(place_pose, end_effector_link='joint6_flange')

                place_offset_plan_result = move_group.plan()
                if isinstance(place_offset_plan_result, tuple):
                    place_offset_success, place_offset_plan, _, _ = place_offset_plan_result
                else:
                    place_offset_success = True
                    place_offset_plan = place_offset_plan_result

                if place_offset_success and place_offset_plan is not None:
                    place_offset_execution_succeeded = move_group.execute(place_offset_plan, wait=True)
                    if place_offset_execution_succeeded:
                        rospy.loginfo('Place offset motion completed.')
                    else:
                        rospy.logwarn('Place offset motion reported failure; continuing after execution ended.')
                else:
                    rospy.logerr('Place offset motion planning failed; continuing to detach.')

                gripper_target = 'open'
                gripper_group.set_named_target(gripper_target)
                rospy.loginfo('Opening gripper.')
                gripper_execution_succeeded = gripper_group.go(wait=True)
                if gripper_execution_succeeded:
                    rospy.loginfo('Gripper motion to the %s state completed.', gripper_target)
                else:
                    rospy.logwarn('Gripper motion reported failure; continuing after execution ended.')

                try:
                    detach_service_name = '/link_attacher_node/detach'
                    rospy.wait_for_service(detach_service_name, timeout=10.0)
                    detach_service = rospy.ServiceProxy(detach_service_name, Attach)
                    detach_request = AttachRequest(
                        model_name_1='firefighter',
                        link_name_1='joint6_flange',
                        model_name_2='cube_clone',
                        link_name_2='link_1',
                    )
                    detach_response = detach_service(detach_request)
                    if detach_response.ok:
                        rospy.loginfo('Detached firefighter from cube_clone.')
                    else:
                        rospy.logerr('The link attacher service rejected the detachment.')
                except (rospy.ROSException, rospy.ServiceException) as error:
                    rospy.logerr('Failed to detach links: %s', error)

                rospy.loginfo('Returning to the init_pose.')
                move_group.set_start_state_to_current_state()
                move_group.set_named_target('init_pose')

                return_plan_result = move_group.plan()
                if isinstance(return_plan_result, tuple):
                    return_success, return_plan, _, _ = return_plan_result
                else:
                    return_success = True
                    return_plan = return_plan_result

                if return_success and return_plan is not None:
                    return_execution_succeeded = move_group.execute(return_plan, wait=True)
                    if return_execution_succeeded:
                        rospy.loginfo('Returned to init_pose.')
                    else:
                        rospy.logwarn('Return to init_pose reported failure.')
                else:
                    rospy.logerr('Return to init_pose planning failed.')
            else:
                rospy.logerr('place_planning failed.')
        else:
            rospy.logerr('Init_pose motion planning failed.')
    else:
        rospy.logerr('Motion planning failed!')

    moveit_commander.roscpp_shutdown()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
