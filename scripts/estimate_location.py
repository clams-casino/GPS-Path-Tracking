#!/usr/bin/env python

import rospy
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
import tf

import numpy as np


class OdomGPSEstimator:

	#will probably need tuning on real grizzly
	R = 6.3781e6

	def __init__(self):

		rospy.init_node('location_estimate', anonymous=True)
		rospy.Subscriber('/navsat/fix', NavSatFix, self.navSatCallback)
		rospy.Subscriber('/odometry/filtered', Odometry, self.odomCallback)

		self.odom_x = 0  #in odom frame which maybe misaligned with the NS-EW frame
		self.odom_y = 0
		self.gps_x = 0   #in GPS frame which is aligned with the NS-EW frame
		self.gps_y = 0
		self.x = 0       #filtered estimate for x and y in NS-EW frame
		self.y = 0

		self.odom_yaw = 0
		self.pos_cov = np.zeros((2,2))
		self.corrected_pos_cov = np.zeros((2,2))
		#not sure if need covariance for yaw since can't correct it anyways

		# need to be able to find initial yaw relative to the NS EW frame through
		# some calibration procedure

		
		#GPS coordinates of starting location
		self.init_long = None
		self.init_lat = None

		#Stores data from GPS callback, spinning on a faster thread
		self.GPS_long = None
		self.GPS_lat = None

		#Data used for calculations and estimation, at a slower rate
		self.prev_long = None
		self.prev_lat = None
		self.curr_long = None
		self.curr_lat = None


		self.rate = rospy.Rate(50)


	def navSatCallback(self,nav_data):
		self.GPS_long = nav_data.longitude
		self.GPS_lat = nav_data.latitude


	

	def setInitNav(self):
		while True:
			if self.GPS_long != None:
				self.init_long = self.GPS_long
				self.init_lat = self.GPS_lat
				self.prev_long = self.GPS_long
				self.prev_lat = self.GPS_lat
				self.curr_long = self.GPS_long
				self.curr_lat = self.GPS_lat
				print 'initial coordinates set as'
				print 'longitude: ', self.init_long
				print 'latitude: ', self.init_lat
				break


	def gpsCoordinateChange(self):
		#if getting GPS measurements
		if self.GPS_long != None:

			self.curr_long = self.GPS_long
			self.curr_lat = self.GPS_lat

			dlat = (self.curr_lat - self.prev_lat) * np.pi / 180
			dlong = (self.curr_long - self.prev_long) * np.pi / 180

			dgps_x = 2 * self.R * np.arcsin(np.sqrt(np.cos(self.curr_lat)**2 * np.sin(dlong/2)**2))
			dgps_y = 2 * self.R * np.arcsin(np.sqrt(np.sin(dlat/2)**2))

			if abs(self.curr_long - self.prev_long) <= 180:
				self.gps_x += np.sign(dlong) * dgps_x #sign of the change

			else:			
				if np.sign(self.curr_long - self.prev_long) == -1:
					self.gps_x += dgps_x
				else:
					self.gps_x -= dgps_x

			self.gps_y += np.sign(dlat) * dgps_y 


			self.prev_long = self.curr_long
			self.prev_lat = self.curr_lat



	def odomCallback(self,odom_data):
		self.odom_x = odom_data.pose.pose.position.x
		self.odom_y = odom_data.pose.pose.position.y
		self.pos_cov = np.array(odom_data.pose.covariance)

		quat = odom_data.pose.pose.orientation
		euler = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
		self.odom_yaw = euler[2];



	def printEstimate(self):
		

		print'longitude: ', self.curr_long
		print'gps x: ', self.gps_x, '\n'
		print'latitude: ', self.curr_lat
		print'gps y: ', self.gps_y
		print '\n\n'







if __name__ == '__main__':
	try:
		estimator = OdomGPSEstimator()
		estimator.setInitNav()

		while not rospy.is_shutdown():
			estimator.gpsCoordinateChange()
			estimator.printEstimate()
			estimator.rate.sleep()

	except rospy.ROSInterruptException:
		pass