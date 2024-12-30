#!/usr/bin/env python3
'''
    randy_bday.py: Control routine for Randy's birthday performance.
'''

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

class RandyPerformance:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('randy_bday', anonymous=True)

        # Publishers
        self.pub_bow = rospy.Publisher('/joint_controller/command', Float64, queue_size=10)
        self.pub_move = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Timing and rate
        self.rate = rospy.Rate(10)  # 10 Hz
        rospy.sleep(0.5)  # Allow publisher connections

    # --- ACTIONS ---
    def bow(self):
        rospy.loginfo("Taking a bow...")
        bow_position = Float64()

        # Bow down
        bow_position.data = 0.5
        self.pub_bow.publish(bow_position)
        rospy.sleep(2)

        # Return to upright position
        bow_position.data = 0.0
        self.pub_bow.publish(bow_position)
        rospy.sleep(1)

    def turn(self, direction='left', duration=3):
        rospy.loginfo(f"Turning {direction}...")
        turn_cmd = Twist()
        turn_cmd.angular.z = 0.5 if direction == 'left' else -0.5

        for _ in range(duration * 10):
            self.pub_move.publish(turn_cmd)
            self.rate.sleep()

        # Stop turning
        self.stop_robot()

    def walk(self, duration=5, speed=0.5):
        rospy.loginfo("Walking...")
        move_cmd = Twist()
        move_cmd.linear.x = speed

        for _ in range(duration * 10):
            self.pub_move.publish(move_cmd)
            self.rate.sleep()

        # Stop walking
        self.stop_robot()

    def stop_robot(self):
        rospy.loginfo("Stopping robot...")
        stop_cmd = Twist()
        self.pub_move.publish(stop_cmd)
        rospy.sleep(1)

    # --- PERFORMANCE ROUTINE ---
    def perform(self):
        rospy.loginfo("Starting Randy's Birthday Performance!")

        # Step 1: Turn toward the audience
        self.turn(direction='left', duration=3)

        # Step 2: Walk to center stage
        self.walk(duration=5)

        # Step 3: Take a bow
        self.bow()

        # Step 4: Walk across stage
        self.walk(duration=10)

        # Step 5: Turn back
        self.turn(direction='right', duration=3)

        # Step 6: Exit stage
        self.walk(duration=5)

        rospy.loginfo("Performance complete!")


if __name__ == '__main__':
    try:
        performance = RandyPerformance()
        performance.perform()
    except rospy.ROSInterruptException:
        rospy.logwarn("Performance interrupted!")

