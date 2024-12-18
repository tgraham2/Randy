#!/bin/env python3
'''
    turn back to original orientation
'''

import rospy
from geometry_msgs.msg import Twist

def turn_back(pub):
    turn_cmd = Twist()
    turn_cmd.angular.z = -0.5  # Adjust rotation speed as needed for turning back
    rate = rospy.Rate(10)  # 10 Hz
    duration = 3  # Adjust duration to complete the turn back
    
    for _ in range(duration * 10):
        pub.publish(turn_cmd)
        rate.sleep()

# Standalone test
if __name__ == '__main__':
    try:
        rospy.init_node('turn_back_test', anonymous=True)
        pub_move = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        turn_back(pub_move)
    except rospy.ROSInterruptException:
        pass
