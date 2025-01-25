#!/usr/bin/env python3
# coding=utf8

import rospy
from puppy_control.msg import Velocity, Pose, Gait

ROS_NODE_NAME = 'puppy_walk_test'

# Walking parameters
PuppyMoveForward = {'x': 5, 'y': 0, 'yaw_rate': 0}  # Slow forward motion for stability

# Walk gait configuration
GaitConfig = {
    'overlap_time': 0.1,
    'swing_time': 0.2,
    'clearance_time': 0.3,
    'z_clearance': 5
}

# Walk pose
PuppyPose = {
    'roll': 0,
    'pitch': 0,
    'yaw': 0,
    'height': -12,
    'x_shift': -0.65,
    'stance_x': 0,
    'stance_y': 0
}

# Function for gradual servo reset
def slow_reset(pose_pub, increments=10, delay=0.2):
    initial_pose = {'roll': 0, 'pitch': 0, 'yaw': 0, 'height': -10, 'x_shift': 0.0, 'stance_x': 0, 'stance_y': 0}

    for i in range(1, increments + 1):
        interpolated_pose = {k: initial_pose[k] + (PuppyPose[k] - initial_pose[k]) * (i / increments)
                             for k in PuppyPose}
        pose_pub.publish(**interpolated_pose)
        rospy.sleep(delay)
    print("Servos reset to Walk pose.")

# Cleanup on shutdown
def cleanup():
    PuppyVelocityPub.publish(x=0, y=0, yaw_rate=0)
    print('Stopping robot...')

if __name__ == '__main__':
    rospy.init_node(ROS_NODE_NAME, log_level=rospy.INFO)
    rospy.on_shutdown(cleanup)

    # Publishers
    PuppyPosePub = rospy.Publisher('/puppy_control/pose', Pose, queue_size=1)
    PuppyGaitConfigPub = rospy.Publisher('/puppy_control/gait', Gait, queue_size=1)
    PuppyVelocityPub = rospy.Publisher('/puppy_control/velocity', Velocity, queue_size=1)

    # Servo reset
    print("Resetting servos to Walk pose...")
    slow_reset(PuppyPosePub)

    # Gait setup
    rospy.sleep(0.2)
    PuppyGaitConfigPub.publish(
        overlap_time=GaitConfig['overlap_time'],
        swing_time=GaitConfig['swing_time'],
        clearance_time=GaitConfig['clearance_time'],
        z_clearance=GaitConfig['z_clearance']
    )
    print("Gait configuration complete.")

    # Test walking forward
    print("Walking forward...")
    PuppyVelocityPub.publish(x=PuppyMoveForward['x'], y=PuppyMoveForward['y'], yaw_rate=PuppyMoveForward['yaw_rate'])
    rospy.sleep(12)  # Walk forward for 1 meter at 5 cm/s
    PuppyVelocityPub.publish(x=0, y=0, yaw_rate=0)  # Stop walking
    print("Walk complete.")

