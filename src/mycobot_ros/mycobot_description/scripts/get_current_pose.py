#!/usr/bin/env python3

import rospy
import tf
from sensor_msgs.msg import JointState


def read_firefighter_joint_state():
    rospy.loginfo('Waiting for /firefighter/joint_states...')
    joint_state = rospy.wait_for_message('/firefighter/joint_states', JointState, timeout=10.0)
    return joint_state


def print_joint_states(joint_state):
    print('\n=== Joint States ===')
    for name, position in zip(joint_state.name, joint_state.position):
        print('%s: %.6f rad' % (name, position))
    print('========================\n')


def print_transform_summary(translation, rotation):
    roll, pitch, yaw = tf.transformations.euler_from_quaternion(rotation)

    print('\n=== End Effector Transform ===')
    print('Reference frame: g_base')
    print('End-effector frame: joint6_flange')
    print('Translation (m): x=%.6f, y=%.6f, z=%.6f' % tuple(translation))
    print('Rotation quaternion: x=%.6f, y=%.6f, z=%.6f, w=%.6f' % tuple(rotation))
    print('Rotation RPY (rad): roll=%.6f, pitch=%.6f, yaw=%.6f' % (roll, pitch, yaw))


def main():
    rospy.init_node('get_current_pose', anonymous=True)

    joint_state = read_firefighter_joint_state()
    print_joint_states(joint_state)
    tf_listener = tf.TransformListener()

    try:
        tf_listener.waitForTransform(
            'g_base', 'joint6_flange', rospy.Time(0), rospy.Duration(10.0))
        translation, rotation = tf_listener.lookupTransform(
            'g_base', 'joint6_flange', rospy.Time(0))
    except (tf.Exception, rospy.ROSException) as error:
        rospy.logerr('Could not get TF from g_base to joint6_flange: %s', error)
        return

    print_transform_summary(translation, rotation)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
