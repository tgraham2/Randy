# Pose for bowing
PuppyBowPose = {'roll': 0, 'pitch': math.radians(-10), 'yaw': 0, 'height': -14, 'x_shift': 0.5, 'stance_x': 0, 'stance_y': 0}

print("Bowing...")
PuppyPosePub.publish(**PuppyBowPose)
rospy.sleep(2)  # Hold bow for 2 seconds
PuppyPosePub.publish(**PuppyNeutralPose)  # Return to neutral
rospy.sleep(1)
print("Bow complete.")

