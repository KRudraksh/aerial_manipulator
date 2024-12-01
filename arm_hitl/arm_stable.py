#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Point, Quaternion
from mavros_msgs.msg import OverrideRCIn

class CONTROLLER:
	def __init__(self):
		# Initialize the ROS node
		rospy.init_node('imu_subscriber', anonymous=True)

		self.linear_acceleration = Point()
		self.orientation = Quaternion()
		
		# Subscribers
		self.subscriber = rospy.Subscriber("/mavros/imu/data", Imu, self.imu_callback)
	
		# Publishers
		self.publish_servo = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)
	
	def imu_callback(self, data):
		self.orientation.x = data.orientation.x
		self.orientation.y = data.orientation.y
		self.orientation.z = data.orientation.z
		self.orientation.w = data.orientation.w

	def euler_from_quaternion(self):
		x = self.orientation.x
		y = self.orientation.y
		z = self.orientation.z
		w = self.orientation.w

		# Roll (x-axis rotation)
		t0 = +2.0 * (w * x + y * z)
		t1 = +1.0 - 2.0 * (x * x + y * y)
		self.roll = math.atan2(t0, t1)

		# Pitch (y-axis rotation)
		t2 = +2.0 * (w * y - z * x)
		t2 = +1.0 if t2 > +1.0 else t2
		t2 = -1.0 if t2 < -1.0 else t2
		self.pitch = math.asin(t2)

		# Yaw (z-axis rotation)
		t3 = +2.0 * (w * z + x * y)
		t4 = +1.0 - 2.0 * (y * y + z * z)
		self.yaw = math.atan2(t3, t4)

		self.roll = math.degrees(self.roll)
		self.pitch = math.degrees(self.pitch)
		self.yaw = math.degrees(self.yaw)

	def control_servo(self, servo_pwm_big, servo_pwm_small):
		rc_msg = OverrideRCIn()
		rc_msg.channels = [0] * 18
		rc_msg.channels[6] = servo_pwm_big
		rc_msg.channels[7] = servo_pwm_small

		# Publish the message
		self.publish_servo.publish(rc_msg)

	def stable_arm(self):
		self.euler_from_quaternion()
		self.servo_pwm_big = int((self.pitch/40.0)*500.0 + 1500.0)
		self.servo_pwm_small = 1500
		self.control_servo(self.servo_pwm_big, self.servo_pwm_small)

if __name__ == '__main__':
	try:
		arm = CONTROLLER()
		rate = rospy.Rate(1000)  # Set to 1Hz to print every second

		while not rospy.is_shutdown():
			arm.stable_arm()
			rate.sleep()		

	except rospy.ROSInterruptException:
		pass
