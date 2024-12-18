#!/bin/env python3
''' 
    Randy birthday demo
    1) walk to middle of stage
    2) turn to the audience
    3) take a bow
    4) return to original orientation
    5) continue walking off stage
'''
# main.py

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

# Import your custom modules
from walk_to_center import walk_to_center
from turn_toward_audience import turn_toward_audience
from bow import bow
from walk_across_stage import walk_across_stage
from turn_back import turn_back

if __name__ == '__main__':
    try:
        rospy.init_node('robot_performance', anonymous=True)
        
        # Publisher for movement
        pub_move = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        # Publisher for bowing
        pub_bow = rospy.Publisher('/joint_controller/command', Float64, queue_size=10)
        
        # Execute the performance sequence
        walk_to_center(pub_move)
        turn_toward_audience(pub_move)
        bow(pub_bow)
        turn_back(pub_move)  # Turn back to the original orientation
        walk_across_stage(pub_move)

    except rospy.ROSInterruptException:
        pass
