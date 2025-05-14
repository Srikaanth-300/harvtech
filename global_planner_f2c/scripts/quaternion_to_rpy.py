#!/usr/bin/env python3

''' This script was tried to get heading (in euler angles) from IMU and GPS fusion only for debugging purposes. '''

import rospy
import tf
from nav_msgs.msg import Odometry
from std_msgs.msg import Header, Float64
import math

class EulerPublisher:
    def __init__(self):
        rospy.init_node('euler_publisher', anonymous=True)

        # Publisher
        self.yaw_a = rospy.Publisher('/yaw_angle', Float64, queue_size=10)
        self.pitch_a = rospy.Publisher('/pitch_angle', Float64, queue_size=10)
        self.roll_a = rospy.Publisher('/roll_angle', Float64, queue_size=10)

        # Subscriber
        rospy.Subscriber('/odometry/filtered', Odometry, self.odom_callback)

    def odom_callback(self, msg):
        # Extract quaternion
        q = msg.pose.pose.orientation
        quaternion = (q.x, q.y, q.z, q.w)

        # Convert to euler
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(quaternion)
	
        print(math.degrees(roll),math.degrees(pitch),math.degrees(yaw))
        # Create message and publish
        msg = Float64()
        msg.data = math.degrees(yaw)
        msg_p = Float64()
        msg_p.data = math.degrees(pitch)
        msg_r = Float64()
        msg_r.data = math.degrees(roll)
        self.yaw_a.publish(msg)
        self.pitch_a.publish(msg_p)
        self.roll_a.publish(msg_r)
        
if __name__ == '__main__':
    try:
        ep = EulerPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

