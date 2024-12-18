#!/bin/env python3
'''
    Take a bow
'''

import rospy
from std_msgs.msg import Float64

def bow(pub):
    bow_position = Float64()
    bow_position.data = 0.5  # Example value for bowing position
    pub.publish(bow_position)
    rospy.sleep(2)  # Hold the bow for 2 seconds
    bow_position.data = 0.0  # Return to the original position
    pub.publish(bow_position)

# Standalone test
if __name__ == '__main__':
    try:
        rospy.init_node('bow_test', anonymous=True)
        pub_bow = rospy.Publisher('/joint_controller/command', Float64, queue_size=10)
        bow(pub_bow)
    except rospy.ROSInterruptException:
        pass
