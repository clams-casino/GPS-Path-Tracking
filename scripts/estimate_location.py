#!/usr/bin/env python

import rospy
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import tf

import numpy as np


class OdomGPSEstimator:

	#will probably need tuning on real grizzly
	R = 6.3781e6

	def __init__(self):

		rospy.init_node('location_estimate', anonymous=True)
		rospy.Subscriber('/navsat/fix', NavSatFix, self.navSatCallback)
		rospy.Subscriber('/odometry/filtered', Odometry, self.odomCallback)
		self.vel_pub = rospy.Publisher('/grizzly_velocity_controller/cmd_vel', Twist, queue_size=5)

		self.odom_x = 0  #in odom frame which maybe misaligned with the NS-EW frame
		self.odom_y = 0
		self.gps_x = 0   #in GPS frame which is aligned with the NS-EW frame
		self.gps_y = 0
		self.x = 0       #filtered estimate for x and y in NS-EW frame
		self.y = 0

		self.odom_yaw = 0
		self.theta_odom_world = None

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


	def navSatCallback(self, nav_data):
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




	def haversineFormula(self, new_long, new_lat, old_long, old_lat):

		dlat = np.deg2rad(new_lat - old_lat)
		dlong = np.deg2rad(new_long - old_long)

		dx = 2 * self.R * np.arcsin(np.sqrt(np.cos(np.deg2rad(new_lat))**2 * np.sin(dlong/2)**2))
		dy = 2 * self.R * np.arcsin(np.sqrt(np.sin(dlat/2)**2))

		if abs(dlong) <= 180:
				dx = np.sign(dlong)*dx
		else:			
			if np.sign(dlong) == -1:
				dx = abs(dx)
			else:
				dx = -abs(dx)

		dy = np.sign(dlat)*dy

		return dx, dy

		

	def gpsCoordinateChange(self):
		#if getting GPS measurements
		if self.GPS_long != None:

			self.curr_long = self.GPS_long
			self.curr_lat = self.GPS_lat

			dgps_x, dgps_y = self.haversineFormula(self.curr_long, self.curr_lat, self.prev_long, self.prev_lat)

			self.gps_x += dgps_x
			self.gps_y += dgps_y

			self.prev_long = self.curr_long
			self.prev_lat = self.curr_lat



	def odomCallback(self, odom_data):
		self.odom_x = odom_data.pose.pose.position.x
		self.odom_y = odom_data.pose.pose.position.y
		self.pos_cov = np.array(odom_data.pose.covariance)

		quat = odom_data.pose.pose.orientation
		euler = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
		self.odom_yaw = euler[2];



	def calibrateTheta(self):
		#finds angle that odom frame is rotated from the NS-EW (world) frame
		#should calibrate on relatively flat ground without much in the way

		#needs to use GPS callback data directly

		data_rate = 1
		calibration_speed = 0.5
		calibration_distance = 5 #[m]

		cal_rate = rospy.Rate(data_rate)

		cal_data = np.zeros([2, calibration_distance/calibration_speed * data_rate])
		cal_cmd_vel = Twist()

		print 'beginning calibration data collection'

		for column in xrange(cal_data.shape[1]):
			cal_cmd_vel.linear.x = calibration_speed;
			vel_pub.publish(cal_cmd_vel)
			cal_rate.sleep()

			cal_data[1, column] = self.GPS_long
			cal_data[2, column] = self.GPS_lat

		cal_cmd_vel.linear.x = 0
		vel_pub.publish(cal_cmd_vel)
		print 'finished calibration data collection, determining odom to world frame orientation'
		print cal_data
		sum = 0
		for column in xrange(cal_data.shape[1]):
			dx, dy = self.haversineFormula(cal_data[1,i], cal_data[2,i], self.init_long, self.init_lat)
			sum += np.arctan2(dy,dx)

		self.theta_odom_world = sum / cal_data.shape[1]
		print 'odom frame is ', theta * np.pi / 180, ' degrees from the NS-EW (world) frame'

		#have it drive back?



		#result initialize attributes for GPS tracking
		self.prev_long = self.GPS_long
		self.prev_lat = self.GPS_lat
		self.curr_long = self.GPS_long
		self.curr_lat = self.GPS_lat



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

		#estimator.calibrateTheta()

		while not rospy.is_shutdown():
			estimator.gpsCoordinateChange()
			estimator.printEstimate()
			estimator.rate.sleep()

	except rospy.ROSInterruptException:
		pass