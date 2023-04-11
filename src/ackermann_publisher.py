#!/usr/bin/env python3
import rospy
import getch
import sensor_msgs.msg
from ackermann_msgs.msg import AckermannDriveStamped
#import sensor_msgs from Joy

def key_ackermann():
	rospy.init_node('key_to_joy')
	Joy_set = sensor_msgs.msg.Joy()
	# pub = rospy.Publisher('joy', sensor_msgs.msg.Joy, queue_size=10)
	pub = rospy.Publisher("/drive", AckermannDriveStamped, queue_size=1)
	cmd = AckermannDriveStamped()
	rate = rospy.Rate(5)
	while not rospy.is_shutdown():
		k = ord(getch.getch())
#		print(str(k))
		print(chr(k))
		if (chr(k) == 'q'):
			rospy.loginfo("Exit")
			exit()
		elif (chr(k) == 'z'): # pause
			# Joy_set.axes = [0, 0, 0, 0, 0, 0, 0, 0]
			cmd.drive.steering_angle = 0.0
			cmd.drive.steering_angle_velocity = 0.0
			cmd.drive.speed = 0.0
			cmd.drive.acceleration = 0.0
			cmd.drive.jerk = 0.0
		elif (chr(k) == 'w'): # foward
			# Joy_set.axes = [0, 1, 0, 0, 0, 0, 0, 0]
			cmd.drive.steering_angle = 0.0
			cmd.drive.steering_angle_velocity = 0.0
			cmd.drive.speed = 0.15
			cmd.drive.acceleration = 0.0
			cmd.drive.jerk = 0.0
		elif (chr(k) == 's'): # reverse
			# Joy_set.axes = [0, -1, 0, 0, 0, 0, 0, 0]
			cmd.drive.steering_angle = 0.0
			cmd.drive.steering_angle_velocity = 0.0
			cmd.drive.speed = -0.1
			cmd.drive.acceleration = 0.0
			cmd.drive.jerk = 0.0
		elif (chr(k) == 'a'): # 
			# Joy_set.axes = [0, 1, 1, 0, 0, 0, 0, 0]
			cmd.drive.steering_angle = 0.25
			cmd.drive.steering_angle_velocity = 0.0
			cmd.drive.speed = 0.1
			cmd.drive.acceleration = 0.0
			cmd.drive.jerk = 0.0
		elif (chr(k) == 'd'): # 
			# Joy_set.axes = [0, 1, -1, 0, 0, 0, 0, 0]
			cmd.drive.steering_angle = -0.25
			cmd.drive.steering_angle_velocity = 0.0
			cmd.drive.speed = 0.1
			cmd.drive.acceleration = 0.0
			cmd.drive.jerk = 0.0
		# print(Joy_set)
		# pub.publish(Joy_set)
		pub.publish(cmd)

if __name__ == '__main__':
	try:
		key_ackermann()
	except rospy.ROSInterruptException:
		pass
