#!/usr/bin/env python3
# coding=utf8

import rospy
from puppy_control.msg import Pose

ROS_NODE_NAME = 'puppy_reset_test'

def slow_reset(pose_pub, increments=10, delay=0.2):
    """
    Gradually reset servos to the neutral position.
    :param pose_pub: ROS publisher for pose messages.
    :param increments: Number of steps for interpolation.
    :param delay: Delay between each step (in seconds).
    """
    initial_pose = {'roll': 0, 'pitch': 0, 'yaw': 0, 'height': -10, 'x_shift': 0.0, 'stance_x': 0, 'stance_y': 0}
    target_pose = {'roll': 0, 'pitch': 0, 'yaw': 0, 'height': -12, 'x_shift': 0.5, 'stance_x': 0, 'stance_y': 0}

    for i in range(1, increments + 1):
        interpolated_pose = {k: initial_pose[k] + (target_pose[k] - initial_pose[k]) * (i / increments)
                             for k in initial_pose}
        pose_pub.publish(**interpolated_pose)
        rospy.sleep(delay)
    print("Servos reset to neutral position.")

if __name__ == '__main__':
    rospy.init_node(ROS_NODE_NAME, log_level=rospy.INFO)

    # Publisher
    PuppyPosePub = rospy.Publisher('/puppy_control/pose', Pose, queue_size=1)

    print("Resetting servos...")
    slow_reset(PuppyPosePub)

    print("Servo reset complete.")

