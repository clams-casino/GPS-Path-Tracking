#!/usr/bin/env python

import rospy
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
import tf

import numpy as np


class OdomGPSEstimator:

	R = 6.371e6
	theta = 0  #amount the robot starting frame is rotated CCW from NS-EW reference frame

	def __init__(self):

		rospy.init_node('location_estimate', anonymous=True)
		rospy.Subscriber('/navsat/fix', NavSatFix, self.navSatCallback)
		rospy.Subscriber('/odometry/filtered', Odometry, self.odomCallback)

		self.odom_x = 0
		self.odom_y = 0
		self.gps_x = 0
		self.gps_y = 0
		self.x = 0
		self.y = 0

		self.yaw = 0
		self.pos_cov = np.zeros((2,2))
		self.corrected_pos_cov = np.zeros((2,2))
		#not sure if need covariance for yaw since can't correct it anyways

		

		self.init_long = None
		self.init_lat = None
		self.long = None
		self.lat = None




	def navSatCallback(self,nav_data):
		self.long = nav_data.longitude
		self.lat = nav_data.latitude




	def setInitNav(self):
		while True:
			if self.long != None:
				self.init_long = self.long
				self.init_lat = self.lat
				print 'initial coordinates set as'
				print 'longitude: ', self.init_long
				print 'latitude: ', self.init_lat
				break


	def gpsCoordinateChange(self):
		if self.long != None:
			dlat = self.lat - self.init_lat
			dlong = self.long - self.init_long

			

			#need to get xy change not just distance change


	def odomCallback(self,odom_data):
		self.odom_x = odom_data.pose.pose.position.x
		self.odom_y = odom_data.pose.pose.position.y
		self.pos_cov = np.array(odom_data.pose.covariance)

		quat = odom_data.pose.pose.orientation
		euler = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
		self.yaw = euler[2];



	def printEstimate(self):
		print'x: ', self.x
		print'y: ', self.y
		print'yaw: ', self.yaw

		print'latitude: ', self.lat
		print'longitude: ', self.long

		print self.pos_cov






if __name__ == '__main__':
	try:
		estimator = OdomGPSEstimator()
		estimator.setInitNav()

		while not rospy.is_shutdown():
			print estimator.printEstimate()

	except rospy.ROSInterruptException:
		pass