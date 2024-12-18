#!/bin/env python3
'''
    Turn toward the audience
'''

# turn_toward_audience.py

import rospy
from geometry_msgs.msg import Twist

def turn_toward_audience(pub):
    turn_cmd = Twist()
    turn_cmd.angular.z = 0.5  # Adjust rotation speed as needed
    rate = rospy.Rate(10)  # 10 Hz
    duration = 3  # Adjust duration to complete the turn

    for _ in range(duration * 10):
        pub.publish(turn_cmd)
        rate.sleep()

# Standalone test
if __name__ == '__main__':
    try:
        rospy.init_node('turn_toward_audience_test', anonymous=True)
        pub_move = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        turn_toward_audience(pub_move)
    except rospy.ROSInterruptException:
        pass

