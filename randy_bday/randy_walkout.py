#!/usr/bin/env python3
# coding=utf8

import math
import rospy
from puppy_control.msg import Velocity, Pose, Gait

ROS_NODE_NAME = 'puppy_salutation_demo'

# Define walking parameters
PuppyMoveForward = {'x': 2, 'y': 0, 'yaw_rate': 0}  # Smaller step forward, 2 cm/s
PuppyTurnLeft = {'x': 0, 'y': 0, 'yaw_rate': math.radians(30) / 3}  # Slower turn, 30° over 3 seconds
PuppyTurnRight = {'x': 0, 'y': 0, 'yaw_rate': -math.radians(30) / 3}

# Adjusted gait configuration for stability
GaitConfig = {
    'overlap_time': 0.6,  # More overlap for stability
    'swing_time': 0.7,    # Slower swing for precise movement
    'clearance_time': 0.1,
    'z_clearance': 2      # Lower foot lift for better stability
}

# Adjusted poses
PuppyBowPose = {'roll': 0, 'pitch': math.radians(-10), 'yaw': 0, 'height': -15, 'x_shift': 0.5, 'stance_x': 0, 'stance_y': 0}
PuppyNeutralPose = {'roll': 0, 'pitch': 0, 'yaw': 0, 'height': -14, 'x_shift': 0.5, 'stance_x': 0, 'stance_y': 0}

def cleanup():
    PuppyVelocityPub.publish(x=0, y=0, yaw_rate=0)
    print('Stopping robot...')

def slow_reset(pose_pub, increments=15, delay=0.2):
    """
    Gradually reset servos to the neutral position.
    """
    initial_pose = {'roll': 0, 'pitch': 0, 'yaw': 0, 'height': -10, 'x_shift': 0.0, 'stance_x': 0, 'stance_y': 0}
    target_pose = PuppyNeutralPose

    for i in range(1, increments + 1):
        interpolated_pose = {k: initial_pose[k] + (target_pose[k] - initial_pose[k]) * (i / increments)
                             for k in initial_pose}
        pose_pub.publish(**interpolated_pose)
        rospy.sleep(delay)
    print("Servos reset to neutral position.")

if __name__ == '__main__':
    rospy.init_node(ROS_NODE_NAME, log_level=rospy.INFO)
    rospy.on_shutdown(cleanup)

    # Publishers
    PuppyPosePub = rospy.Publisher('/puppy_control/pose', Pose, queue_size=1)
    PuppyGaitConfigPub = rospy.Publisher('/puppy_control/gait', Gait, queue_size=1)
    PuppyVelocityPub = rospy.Publisher('/puppy_control/velocity', Velocity, queue_size=1)

    # Servo reset
    print("Resetting servos to neutral...")
    slow_reset(PuppyPosePub)

    # Gait setup
    rospy.sleep(0.2)
    PuppyGaitConfigPub.publish(
        overlap_time=GaitConfig['overlap_time'],
        swing_time=GaitConfig['swing_time'],
        clearance_time=GaitConfig['clearance_time'],
        z_clearance=GaitConfig['z_clearance']
    )

    # 1. Walk forward
    print("Walking forward...")
    PuppyVelocityPub.publish(x=PuppyMoveForward['x'], y=PuppyMoveForward['y'], yaw_rate=PuppyMoveForward['yaw_rate'])
    rospy.sleep(15)  # Walk for 30 cm

    # 2. Turn left and bow
    print("Turning left...")
    PuppyVelocityPub.publish(x=PuppyTurnLeft['x'], y=PuppyTurnLeft['y'], yaw_rate=PuppyTurnLeft['yaw_rate'])
    rospy.sleep(5)  # Turn 90 degrees

    print("Bowing...")
    PuppyPosePub.publish(**PuppyBowPose)
    rospy.sleep(3)

    print("Returning to neutral...")
    PuppyPosePub.publish(**PuppyNeutralPose)
    rospy.sleep(2)

    # 3. Turn right and walk off
    print("Turning right...")
    PuppyVelocityPub.publish(x=PuppyTurnRight['x'], y=PuppyTurnRight['y'], yaw_rate=PuppyTurnRight['yaw_rate'])
    rospy.sleep(5)  # Turn 90 degrees

    print("Walking forward to exit...")
    PuppyVelocityPub.publish(x=PuppyMoveForward['x'], y=PuppyMoveForward['y'], yaw_rate=PuppyMoveForward['yaw_rate'])
    rospy.sleep(15)  # Walk for 30 cm

    print("Routine complete.")

