#!/bin/env python3
'''
    walk (across) off the stage
'''

import rospy
from geometry_msgs.msg import Twist

def walk_across_stage(pub):
    move_cmd = Twist()
    move_cmd.linear.x = 0.5  # Adjust speed as needed
    rate = rospy.Rate(10)  # 10 Hz
    duration = 10  # Adjust duration to complete the trek across the stage
    
    for _ in range(duration * 10):
        pub.publish(move_cmd)
        rate.sleep()

# Standalone test
if __name__ == '__main__':
    try:
        rospy.init_node('walk_across_stage_test', anonymous=True)
        pub_move = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        walk_across_stage(pub_move)
    except rospy.ROSInterruptException:
        pass
