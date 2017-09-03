#!/usr/bin/env python


import rospy
from geometry_msgs.msg import Twist, Pose, PoseWithCovarianceStamped, PoseStamped, PointStamped
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseGoal
import math

"""
ROS Node that intercepts the Twist msgs coming from the move_base action interface and throttles it when required.

Author: Siddharth Agrawal

"""

class VelocityScaler(object):
	def __init__(self):
		rospy.loginfo("Initialising Velocity Scaler..")

		#check how the move_base wants to command the base.
		self.move_base_vel_sub = rospy.Subscriber('/mobile_base/commands/velocity', Twist, self.move_base_vel_cb, queue_size=1)

		#publish scaled velocity to the base.
		self.scaled_vel_pub = rospy.Publisher('/mobile_base/commands/velocity_final', Twist, queue_size=1)

		#for continuously tracking the current pose in the map.
		self.amcl_pose_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_cb, queue_size=1)

		#for getting the current goal to the move_base.
		self.move_base_goal_sub = rospy.Subscriber('/move_base/goal', MoveBaseActionGoal, self.move_base_goal_cb, queue_size=1)

		#How far from the goal we want to throttle the velocity.
		self.distance_threshold_throttling = 2.0
		self.last_move_base_vel = None		#last Twist msg from move_base
		self.last_amcl_pose = None			#current amcl_pose
		self.move_base_goal = None			#current move_base_goal
		self.is_last_goal_reached = False	
		#self.distance_from_goal = 10000		#initialised at a random high value
		rospy.loginfo("Velocity scaler objects initialised...")
		rospy.sleep(5.0)
		rospy.loginfo("Ready to receive velocity commands now..")


	def move_base_vel_cb(self, data):
		self.last_move_base_vel = data

	def amcl_cb(self, data):
		self.last_amcl_pose = data.pose.pose.position

	def move_base_goal_cb(self,data):
		self.move_base_goal = data.goal.target_pose.pose.position


	def getDistance(self):
		return math.sqrt((self.last_amcl_pose.x-self.move_base_goal.x)**2 +(self.last_amcl_pose.y-self.move_base_goal.y)**2)


	def do_work(self):
		if(self.move_base_goal != None and self.last_move_base_vel != None):
			if(self.getDistance() > self.distance_threshold_throttling):
				self.scaled_vel_pub.publish(self.last_move_base_vel)
			else:
				self.scaled_vel_pub.publish(self.last_move_base_vel)
				rospy.loginfo("Velocity being throttled..")





	def run(self):
		r = rospy.Rate(10)
		while not rospy.is_shutdown():
			self.do_work()
			r.sleep()


if __name__ == '__main__':
	rospy.init_node('velocity_scaler')
	scaler = VelocityScaler()
	scaler.run()