#!/usr/bin/python3
import rospy
from nav_msgs.msg import Odometry
import csv
import os

class get_points():

    def __init__(self):
        
        rospy.init_node("xy_coordinates")
        rospy.Subscriber("/odometry/filtered", Odometry, self.odom_callback)
        self.points = []
        rospy.on_shutdown(self.save_to_csv)
        self.csv_file = os.path.expanduser("~/harvtech_ws/src/navigation/csv_files/world_xy_values/simulated_boundary.csv")


    def odom_callback(self,msg):

        x_world = msg.pose.pose.position.x
        y_world = msg.pose.pose.position.y

        self.points.append((x_world,y_world))

    def save_to_csv(self):
    
        rospy.loginfo("Node shutting down. Saving odometry to CSV...")

        with open(self.csv_file, 'w') as f:
            writer = csv.writer(f)
            writer.writerows(self.points)


if __name__ == '__main__':
    get_points()
    rospy.spin()
