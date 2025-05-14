#!/usr/bin/env python3

'''This script is to publish a dummy goal point to start move_base '''

import rospy
from geometry_msgs.msg import PoseStamped

def publish_goal():

    pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

    rospy.sleep(1.0)

    goal = PoseStamped()
    goal.header.stamp = rospy.Time.now()
    goal.header.frame_id = "map"
    goal.pose.position.x = 0.0
    goal.pose.position.y = 0.0
    goal.pose.position.z = 0.0
    goal.pose.orientation.w = 1.0

    rospy.loginfo("Starting the plan")
    pub.publish(goal)

if __name__ == "__main__":
	rospy.init_node("goal_initial")
	publish_goal()
