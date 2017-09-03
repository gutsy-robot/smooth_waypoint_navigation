#!/usr/bin/env python

import rospy
import actionlib
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from nav_msgs.msg import Path 
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import *



class WayPointNav(object):
	def __init__(self):
		rospy.loginfo("Initialising parameters....")
		self.point_pub = rospy.Publisher(
            '/nav_points', PointStamped, queue_size=1)
		self.waypoints = None
		self.current_goal_number = 0
		self.total_waypoints = 0
		self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
		rospy.loginfo("wait for the action server to come up")
	#allow up to 5 seconds for the action server to come up
		self.move_base.wait_for_server(rospy.Duration(5))
		#self.create_waypoints()
		rospy.loginfo("Waypoints created..")
		rospy.sleep(4.0)


	def create_waypoints(self):
		for i in range(0,3):
			waypoint = PointStamped()
			waypoint.header.frame_id = "map"
			waypoint.point. x = i
			self.waypoints[i] = waypoint

		self.total_waypoints =3 


	def do_work(self):
		#self.point_pub.publish(waypoints[current_goal_number])
		goal = MoveBaseGoal()
		goal.target_pose.header.frame_id = 'map'
		goal.target_pose.header.stamp = rospy.Time.now()
		goal.target_pose.pose.position.x = 4.0
		goal.target_pose.pose.position.y = -4.0
		goal.target_pose.pose.orientation.w = 1.0 #go forward

	
        	self.move_base.send_goal(goal)

	#allow TurtleBot up to 60 seconds to complete task
		success = self.move_base.wait_for_result(rospy.Duration(60)) 


		if not success:
                	self.move_base.cancel_goal()
                	rospy.loginfo("The base failed to move the waypoint")
                	self.current_goal_number += 1
    		else:
		# We made it!
			state = self.move_base.get_state()
			if state == GoalStatus.SUCCEEDED:
		    		rospy.loginfo("Hooray, the base moved to the waypoint")
		    		'''
		    		if(self.current_goal_number <= self.total_waypoints -1):
		    			self.current_goal_number +=1 
		    		else:
		    			rospy.loginfo("all waypoints travelled..")

					'''



	def run(self):
		r = rospy.Rate(10)
		while not rospy.is_shutdown():
			self.do_work()
			r.sleep()




if __name__== '__main__':
		rospy.init_node('smooth_waypoint_navigation')
		nav = WayPointNav()
		nav.run()
