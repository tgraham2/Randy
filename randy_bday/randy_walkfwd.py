#!/usr/bin/env python3
# coding=utf8

import rospy
from puppy_control.msg import Velocity, Pose, Gait

ROS_NODE_NAME = 'puppy_walk_test'

# Define walking parameters
PuppyMoveForward = {'x': 5, 'y': 0, 'yaw_rate': 0}  # Slow forward motion

def slow_reset(pose_pub, increments=10, delay=0.2):
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

    # Publishers
    PuppyPosePub = rospy.Publisher('/puppy_control/pose', Pose, queue_size=1)
    PuppyVelocityPub = rospy.Publisher('/puppy_control/velocity', Velocity, queue_size=1)

    print("Resetting servos...")
    slow_reset(PuppyPosePub)

    print("Walking forward...")
    PuppyVelocityPub.publish(x=PuppyMoveForward['x'], y=PuppyMoveForward['y'], yaw_rate=PuppyMoveForward['yaw_rate'])
    rospy.sleep(12)  # Walk for 1 meter at 5 cm/s
    PuppyVelocityPub.publish(x=0, y=0, yaw_rate=0)  # Stop
    print("Walk complete.")

