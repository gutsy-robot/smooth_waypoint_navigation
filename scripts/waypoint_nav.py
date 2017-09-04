#!/usr/bin/env python

import rospy
import actionlib
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from nav_msgs.msg import Path 
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import *
import math



class WayPointNav(object):
	def __init__(self):
		rospy.loginfo("Initialising parameters....")
		self.waypoints_x = [9.0,9.0,1.0]
		self.waypoints_y = [1.0,5.0,9.0]
		self.current_goal_number = 0
		self.total_waypoints = 3
		self.is_first_waypoint_sent = False
		self.waypoint_cleared_threshold = 1.0
		self.last_amcl_pose = None
		self.amcl_pose_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_cb, queue_size=1)
		self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
		rospy.loginfo("wait for the action server to come up")
	#allow up to 5 seconds for the action server to come up
		self.move_base.wait_for_server(rospy.Duration(5))
		#self.create_waypoints()
		rospy.loginfo("Waypoints created..")
		rospy.sleep(7.0)

	def amcl_cb(self,data):
		self.last_amcl_pose = data.pose.pose.position

	def getDistance(self):
		return math.sqrt((self.last_amcl_pose.x-self.waypoints_x[self.current_goal_number-1])**2 +(self.last_amcl_pose.y-self.waypoints_y[self.current_goal_number-1])**2)



	def do_work(self):
		#self.point_pub.publish(waypoints[current_goal_number])
		if(self.is_first_waypoint_sent == False):
			rospy.loginfo("Sending first goal")
			goal = MoveBaseGoal()
			goal.target_pose.header.frame_id = 'map'
			goal.target_pose.header.stamp = rospy.Time.now()
			goal.target_pose.pose.position.x = self.waypoints_x[self.current_goal_number]
			goal.target_pose.pose.position.y = self.waypoints_y[self.current_goal_number]
			goal.target_pose.pose.orientation.w = 1.0 #go forward
			self.move_base.send_goal(goal)
			self.current_goal_number +=1
			self.is_first_waypoint_sent = True
			rospy.loginfo("First goal sent...")

		else:
			if(self.getDistance() < self.waypoint_cleared_threshold and self.current_goal_number < self.total_waypoints):
				rospy.loginfo("Sending next goal...")
				goal = MoveBaseGoal()
				goal.target_pose.header.frame_id = 'map'
				goal.target_pose.header.stamp = rospy.Time.now()
				goal.target_pose.pose.position.x = self.waypoints_x[self.current_goal_number]
				goal.target_pose.pose.position.y = self.waypoints_y[self.current_goal_number]
				goal.target_pose.pose.orientation.w = 1.0 #go forward
				self.move_base.send_goal(goal)
				self.current_goal_number +=1
				rospy.loginfo("goal sent...")




	def run(self):
		r = rospy.Rate(10)
		while not rospy.is_shutdown():
			self.do_work()
			r.sleep()




if __name__== '__main__':
		rospy.init_node('smooth_waypoint_navigation')
		nav = WayPointNav()
		nav.run()
