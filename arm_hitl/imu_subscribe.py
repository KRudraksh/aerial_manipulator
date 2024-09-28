#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Point, Quaternion

class CONTROLLER:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('imu_subscriber', anonymous=True)
        
        # Initialize the Point variable to store linear acceleration data
        self.linear_acceleration = Point()
        self.orientation = Quaternion()
        
        # Subscribe to the /mavros/imu/data topic
        self.subscriber = rospy.Subscriber("/mavros/imu/data", Imu, self.imu_callback)
    
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

if __name__ == '__main__':
    try:
        controller = CONTROLLER()
        rate = rospy.Rate(10)  # Set to 1Hz to print every second
       
        while not rospy.is_shutdown():
            
            # orientation = controller.orientation
            # print("Orientation x:", orientation.x, " y:", orientation.y, " z:", orientation.z, " w:", orientation.w)
            # # print("Orientation -> x: {:.2f}, y: {:.2f}, z: {:.2f}, w: {:,2f}".format(
            #     # orientation.x, orientation.y, orientation.z, orientation.w))

            controller.euler_from_quaternion()
            print("Roll:", controller.roll, " Pitch:", controller.pitch, " Yaw:", controller.yaw)

            rate.sleep()

    except rospy.ROSInterruptException:
        pass
