#!/usr/bin/env python3

import copy
import math
import select
import sys
import termios
import tty

import moveit_commander
import rospy
from gazebo_ros_link_attacher.srv import Attach, AttachRequest
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetPositionFK, GetPositionFKRequest
from sensor_msgs.msg import JointState


REFERENCE_FRAME = 'g_base'
END_EFFECTOR_LINK = 'joint6_flange'
STEP_SIZE = 0.01
EEF_STEP = 0.01
JOINT_STEP = math.radians(5.0)
GRIPPER_STATE_TOLERANCE = 0.005
ROBOT_MODEL = 'firefighter'
ROBOT_LINK = 'joint6_flange'
#OBJECT_MODEL = input("Enter the object model name:")
OBJECT_LINK = 'link_1'

KEY_BINDINGS = {
	'w': ('z', STEP_SIZE, 'Z+'),
	's': ('z', -STEP_SIZE, 'Z-'),
	'a': ('x', -STEP_SIZE, 'X-'),
	'd': ('x', STEP_SIZE, 'X+'),
	'r': ('y', STEP_SIZE, 'Y+'),
	'f': ('y', -STEP_SIZE, 'Y-'),
}


def get_live_robot_state(robot, move_group, joint_state):
	robot_state = robot.get_current_state()
	positions_by_name = dict(zip(joint_state.name, joint_state.position))
	state_position_indices = {
		name: index for index, name in enumerate(robot_state.joint_state.name)
	}

	missing_joint_names = [
		name for name in move_group.get_joints()
		if name not in positions_by_name or name not in state_position_indices
	]
	if missing_joint_names:
		rospy.logerr('Live joint state is missing arm joints: %s', missing_joint_names)
		return None

	state_positions = list(robot_state.joint_state.position)
	for name in move_group.get_joints():
		state_positions[state_position_indices[name]] = positions_by_name[name]
	robot_state.joint_state.position = state_positions
	return robot_state


def get_current_pose(robot_state, compute_fk):
	request = GetPositionFKRequest()
	request.header.frame_id = REFERENCE_FRAME
	request.fk_link_names = [END_EFFECTOR_LINK]
	request.robot_state = robot_state

	response = compute_fk(request)
	if response.error_code.val != MoveItErrorCodes.SUCCESS:
		rospy.logerr('Forward kinematics failed with MoveIt error code %d.', response.error_code.val)
		return None
	if not response.pose_stamped:
		rospy.logerr('Forward kinematics returned no pose for %s.', END_EFFECTOR_LINK)
		return None
	return response.pose_stamped[0].pose


def move_cartesian_step(move_group, robot, compute_fk, axis, distance):
	try:
		joint_state = rospy.wait_for_message(
			'/firefighter/joint_states',
			JointState,
			timeout=5.0,
		)
	except rospy.ROSException as error:
		rospy.logerr('Could not read live Gazebo joint state: %s', error)
		return

	robot_state = get_live_robot_state(robot, move_group, joint_state)
	if robot_state is None:
		return

	current_pose = get_current_pose(robot_state, compute_fk)
	if current_pose is None:
		return

	target_pose = copy.deepcopy(current_pose)
	setattr(target_pose.position, axis, getattr(target_pose.position, axis) + distance)
	move_group.set_start_state(robot_state)

	waypoints = [copy.deepcopy(current_pose), copy.deepcopy(target_pose)]
	plan, fraction = move_group.compute_cartesian_path(
		waypoints,
		EEF_STEP,
		avoid_collisions=True,
	)
	rospy.loginfo('Cartesian path completion: %.1f%%', fraction * 100.0)
	if fraction < 1.0:
		rospy.logwarn('Jog path is incomplete. Not executing this key press.')
		return

	if len(plan.joint_trajectory.points) > 1:
		plan.joint_trajectory.points = plan.joint_trajectory.points[1:]

	timed_plan = move_group.retime_trajectory(
		robot_state,
		plan,
		0.1,
		0.1,
	)
	if not timed_plan.joint_trajectory.points:
		rospy.logerr('Trajectory time parameterization failed; not executing the jog.')
		return

	execution_succeeded = move_group.execute(timed_plan, wait=True)
	move_group.stop()
	if execution_succeeded:
		rospy.loginfo('Cartesian jog completed.')
	else:
		rospy.logwarn('Controller reported failure for this Cartesian jog.')


def move_joint_step(move_group, robot, joint_index, direction):
	try:
		joint_state = rospy.wait_for_message(
			'/firefighter/joint_states',
			JointState,
			timeout=5.0,
		)
	except rospy.ROSException as error:
		rospy.logerr('Could not read live Gazebo joint state: %s', error)
		return

	robot_state = get_live_robot_state(robot, move_group, joint_state)
	if robot_state is None:
		return

	active_joint_names = move_group.get_active_joints()
	if joint_index >= len(active_joint_names):
		rospy.logerr('Arm group has only %d active joints.', len(active_joint_names))
		return

	positions_by_name = dict(zip(joint_state.name, joint_state.position))
	target_joints = [positions_by_name[name] for name in active_joint_names]
	joint_name = active_joint_names[joint_index]
	target_joints[joint_index] += direction * JOINT_STEP

	move_group.set_start_state(robot_state)
	move_group.set_joint_value_target(target_joints)
	plan_result = move_group.plan()
	if isinstance(plan_result, tuple):
		planning_succeeded, plan, _, _ = plan_result
	else:
		plan = plan_result
		planning_succeeded = plan is not None and bool(plan.joint_trajectory.points)

	if not planning_succeeded or plan is None or not plan.joint_trajectory.points:
		rospy.logwarn('Planning failed for %s; joint was not moved.', joint_name)
		return

	timed_plan = move_group.retime_trajectory(robot_state, plan, 0.1, 0.1)
	if not timed_plan.joint_trajectory.points:
		rospy.logerr('Trajectory retiming failed for %s.', joint_name)
		return

	execution_succeeded = move_group.execute(timed_plan, wait=True)
	move_group.stop()
	if execution_succeeded:
		rospy.loginfo('Joint %d (%s) moved by %.1f degrees.', joint_index + 1, joint_name, direction * 5.0)
	else:
		rospy.logwarn('Controller reported failure while moving joint %d (%s).', joint_index + 1, joint_name)


def toggle_gripper(gripper_group):
	try:
		joint_state = rospy.wait_for_message(
			'/firefighter/joint_states',
			JointState,
			timeout=5.0,
		)
	except rospy.ROSException as error:
		rospy.logerr('Could not read gripper state from Gazebo: %s', error)
		return

	open_targets = gripper_group.get_named_target_values('open')
	close_targets = gripper_group.get_named_target_values('close')
	positions_by_name = dict(zip(joint_state.name, joint_state.position))
	gripper_joints = gripper_group.get_active_joints()
	missing_joint_names = [
		name for name in gripper_joints
		if name not in positions_by_name or name not in open_targets or name not in close_targets
	]
	if missing_joint_names:
		rospy.logerr('Cannot determine gripper state. Missing joint values for %s.', missing_joint_names)
		return

	open_error = sum(
		abs(positions_by_name[name] - open_targets[name])
		for name in gripper_joints
	)
	close_error = sum(
		abs(positions_by_name[name] - close_targets[name])
		for name in gripper_joints
	)

	if open_error <= GRIPPER_STATE_TOLERANCE:
		target_state = 'close'
	elif close_error <= GRIPPER_STATE_TOLERANCE:
		target_state = 'open'
	else:
		target_state = 'close' if open_error <= close_error else 'open'
		rospy.loginfo('Gripper is between named states; switching to %s.', target_state)

	gripper_group.set_named_target(target_state)
	execution_succeeded = gripper_group.go(wait=True)
	gripper_group.stop()
	if execution_succeeded:
		rospy.loginfo('Gripper moved to the %s state.', target_state)
	else:
		rospy.logwarn('Gripper controller reported failure moving to %s.', target_state)


def toggle_attachment(object_attached):
	service_action = 'detach' if object_attached else 'attach'
	service_name = '/link_attacher_node/%s' % service_action
	select_cube = input("Select the cube to attach/detach (1.cube, 2.cube_clone, 3.ube_clone_0): ")
	if select_cube == '1':
		OBJECT_MODEL = 'cube_clone_0'
	elif select_cube == '2':
		OBJECT_MODEL = 'cube'
	elif select_cube == '3':
		OBJECT_MODEL = 'cube_clone'

	try:
		rospy.wait_for_service(service_name, timeout=5.0)
		link_attacher = rospy.ServiceProxy(service_name, Attach)
		request = AttachRequest(
			model_name_1=ROBOT_MODEL,
			link_name_1=ROBOT_LINK,
			model_name_2=OBJECT_MODEL,
			link_name_2=OBJECT_LINK,
		)
		response = link_attacher(request)
	except (rospy.ROSException, rospy.ServiceException) as error:
		rospy.logerr('Failed to %s object: %s', service_action, error)
		return object_attached

	if response.ok:
		new_attached_state = not object_attached
		rospy.loginfo('Object %s: %s.', service_action, 'attached' if new_attached_state else 'detached')
		return new_attached_state

	rospy.logerr('Link-attacher service rejected the %s request.', service_action)
	return object_attached


def main():
	moveit_commander.roscpp_initialize(sys.argv)
	rospy.init_node('cartesian_keyboard_joystick', anonymous=True)

	move_group = moveit_commander.MoveGroupCommander('arm_group')
	gripper_group = moveit_commander.MoveGroupCommander('gripper')
	robot = moveit_commander.RobotCommander()
	move_group.set_pose_reference_frame(REFERENCE_FRAME)
	move_group.set_end_effector_link(END_EFFECTOR_LINK)
	move_group.set_planning_time(20.0)
	move_group.allow_replanning(True)

	rospy.wait_for_service('/compute_fk')
	compute_fk = rospy.ServiceProxy('/compute_fk', GetPositionFK)

	if not sys.stdin.isatty():
		rospy.logerr('Keyboard control requires a terminal (TTY).')
		return

	file_descriptor = sys.stdin.fileno()
	original_terminal_settings = termios.tcgetattr(file_descriptor)
	active_joint_names = move_group.get_active_joints()
	selected_joint = None
	object_attached = False
	try:
		tty.setcbreak(file_descriptor)
		print('Cartesian jog: W/S = Z +/-, A/D = X -/+, R/F = Y +/-')
		print('Joint jog: press 1-6 to select a joint, then + or - (5 degrees). Q = quit')
		print('Gripper: SPACE toggles between open and close')
		print('Attachment: T toggles attach/detach')
		while not rospy.is_shutdown():
			readable, _, _ = select.select([sys.stdin], [], [], 0.1)
			if not readable:
				continue

			key = sys.stdin.read(1).lower()
			if key == 'q':
				break
			if key == ' ':
				toggle_gripper(gripper_group)
				termios.tcflush(file_descriptor, termios.TCIFLUSH)
				continue
			if key == 't':
				object_attached = toggle_attachment(object_attached)
				continue
			if key in '123456':
				selected_joint = int(key) - 1
				if selected_joint < len(active_joint_names):
					rospy.loginfo(
						'Selected joint %d: %s. Press + or - to jog it.',
						selected_joint + 1,
						active_joint_names[selected_joint],
					)
				else:
					rospy.logerr('Arm group has only %d active joints.', len(active_joint_names))
					selected_joint = None
				continue
			if key in ('+', '=', '-'):
				if selected_joint is None:
					rospy.logwarn('Select a joint with a number from 1 to 6 first.')
					continue
				direction = 1 if key in ('+', '=') else -1
				move_joint_step(move_group, robot, selected_joint, direction)
				termios.tcflush(file_descriptor, termios.TCIFLUSH)
				continue
			if key not in KEY_BINDINGS:
				continue

			axis, distance, direction = KEY_BINDINGS[key]
			rospy.loginfo('Jogging %s by %.3f m.', direction, abs(distance))
			move_cartesian_step(move_group, robot, compute_fk, axis, distance)
			termios.tcflush(file_descriptor, termios.TCIFLUSH)
	finally:
		termios.tcsetattr(file_descriptor, termios.TCSADRAIN, original_terminal_settings)
		move_group.stop()
		moveit_commander.roscpp_shutdown()


if __name__ == '__main__':
	main()