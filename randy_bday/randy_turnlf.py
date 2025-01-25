# Define turning parameters
PuppyTurnLeft = {'x': 0, 'y': 0, 'yaw_rate': math.radians(45) / 2}  # 45° over 2 seconds

print("Turning left...")
PuppyVelocityPub.publish(x=PuppyTurnLeft['x'], y=PuppyTurnLeft['y'], yaw_rate=PuppyTurnLeft['yaw_rate'])
rospy.sleep(4)  # Turn 90° (2 seconds per 45°)
PuppyVelocityPub.publish(x=0, y=0, yaw_rate=0)  # Stop
print("Turn complete.")

