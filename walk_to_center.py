#!/bin/env python3
'''
    walk to center of stage
'''

import rospy
from geometry_msgs.msg import Twist

def walk_to_center(pub):
    move_cmd = Twist()
    move_cmd.linear.x = 0.5  # Adjust speed as needed
    rate = rospy.Rate(10)  # 10 Hz
    duration = 5  # Adjust duration to reach the center of the stage
    
    for _ in range(duration * 10):
        pub.publish(move_cmd)
        rate.sleep()

# Standalone test
if __name__ == '__main__':
    try:
        rospy.init_node('walk_to_center_test', anonymous=True)
        pub_move = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        walk_to_center(pub_move)
    except rospy.ROSInterruptException:
        pass
