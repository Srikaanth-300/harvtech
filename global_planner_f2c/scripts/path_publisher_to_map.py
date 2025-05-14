#!/usr/bin/env python3

import rospy
import csv
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

def read_csv(file_path):
    """
    Reads a CSV file and returns a list of poses
    each pose as (x, y, z, orientation_w, orientation_x, orientation_y, orientation_z)
    """
    poses = []
    with open(file_path, 'r') as file:
        reader = csv.reader(file, delimiter = ',')
        for row in reader:
            x = float(row[0])
            y = float(row[1])
            poses.append((x, y))
    return poses

def publish_path():
    """
    Reads the pose data from CSV and publishes the path.
    """
    # Initialize the ROS node
    rospy.init_node('path_publisher', anonymous=True)
    
    # Define the publisher for the path message
    path_pub = rospy.Publisher('/robot_path', Path, queue_size=10)

    
    file_path = "/home/srikaanth/harvtech_ws/src/navigation/csv_files/world_xy_values/simulated_boundary.csv"
    
    poses = read_csv(file_path)
    # Create a Path message
    path_msg = Path()
    path_msg.header.frame_id = "map"  # You can change this to your desired frame

    # Add poses to the path message
    for i, pose in enumerate(poses):
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.pose.position.x = pose[0]
        pose_stamped.pose.position.y = pose[1]
        pose_stamped.pose.position.z = 0.0
        pose_stamped.pose.orientation.w = 1.0
        pose_stamped.pose.orientation.x = 0.0
        pose_stamped.pose.orientation.y = 0.0
        pose_stamped.pose.orientation.z = 0.0
        
        # Append the pose to the path
        path_msg.poses.append(pose_stamped)
        
    # Publish the path message
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        path_pub.publish(path_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_path()
    except rospy.ROSInterruptException:
        pass

