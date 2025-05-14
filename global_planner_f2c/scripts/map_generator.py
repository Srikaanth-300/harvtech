#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData,Path
from geometry_msgs.msg import Pose
import numpy as np

class OdomToGridMap:
    def __init__(self):
        rospy.init_node('odom_to_gridmap_node')

        # Map parameters
        self.resolution = 0.05  # meters per cell
        self.width = 1000       # number of cells
        self.height = 1000
        self.origin_x = -25.0  # meters
        self.origin_y = -25.0

        self.map = np.full((self.height, self.width), 0, dtype=np.int8)  # Unknown
        self.occupancy_pub = rospy.Publisher('/odom_grid_map', OccupancyGrid, queue_size=10)
        #rospy.Subscriber('/odometry/filtered', Odometry, self.odom_callback)
        rospy.Subscriber('/robot_path', Path, self.odom_callback)

        rospy.loginfo("Mapping started...")
        rospy.spin()

    def odom_callback(self, msg):
        #x = msg.pose.pose.position.x
        #y = msg.pose.pose.position.y
        for i in range(len(msg.poses)):
          x = msg.poses[i].pose.position.x
          y = msg.poses[i].pose.position.y

          # Convert world coords to map grid coords
          grid_x = int((x - self.origin_x) / self.resolution)
          grid_y = int((y - self.origin_y) / self.resolution)

          if 0 <= grid_x < self.width and 0 <= grid_y < self.height:
              self.map[grid_y][grid_x] = 100  # Mark as occupied

        self.publish_map()

    def publish_map(self):
        grid_msg = OccupancyGrid()
        grid_msg.header.stamp = rospy.Time.now()
        grid_msg.header.frame_id = "map"

        grid_msg.info = MapMetaData()
        grid_msg.info.resolution = self.resolution
        grid_msg.info.width = self.width
        grid_msg.info.height = self.height
        grid_msg.info.origin.position.x = self.origin_x
        grid_msg.info.origin.position.y = self.origin_y
        grid_msg.info.origin.orientation.w = 1.0

        grid_msg.data = self.map.flatten().tolist()
        self.occupancy_pub.publish(grid_msg)

if __name__ == '__main__':
    try:
        OdomToGridMap()
    except rospy.ROSInterruptException:
        pass

