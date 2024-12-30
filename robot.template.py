#!/usr/bin/env python3
'''
    Generic ROS Motion Script
'''

import rospy
from geometry_msgs.msg import Twist

def move_robot(pub, linear_speed, angular_speed, duration):
    rospy.loginfo("Starting motion...")

    # Initialize motion command
    move_cmd = Twist()
    move_cmd.linear.x = linear_speed
    move_cmd.angular.z = angular_speed

    rate = rospy.Rate(10)  # 10 Hz
    for _ in range(int(duration * 10)):
        pub.publish(move_cmd)
        rate.sleep()

    # Stop the robot
    rospy.loginfo("Stopping motion...")
    stop_cmd = Twist()
    pub.publish(stop_cmd)


if __name__ == '__main__':
    try:
        rospy.init_node('motion_test', anonymous=True)

        # Initialize publisher
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.sleep(0.5)  # Ensure publisher connection

        # Load parameters
        linear_speed = rospy.get_param('~linear_speed', 0.5)
        angular_speed = rospy.get_param('~angular_speed', 0.0)
        duration = rospy.get_param('~duration', 5)

        # Execute motion
        move_robot(pub, linear_speed, angular_speed, duration)

    except rospy.ROSInterruptException:
        rospy.logwarn("ROS interrupted. Exiting gracefully.")

