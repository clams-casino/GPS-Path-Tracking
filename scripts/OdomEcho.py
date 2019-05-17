#!/usr/bin/env python

import rospy
import numpy as np
from nav_msgs.msg import Odometry

import tf


def callback(data):

	# get position estimate and covariance
	print 'odometry position'
	print 'x: ', data.pose.pose.position.x
	print 'y: ', data.pose.pose.position.y
	print 'z: ', data.pose.pose.position.z

	print '\n'


	#try to get orientation estimate and covariance

	print 'quaternion: ', data.pose.pose.orientation
	
	quat = data.pose.pose.orientation
	euler = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
	yaw = euler[2];

	print 'yaw in degrees: ', yaw * (180/3.14), '\n'


	np_array = np.array(data.pose.covariance)

	print 'pose covariance', 
	print np_array.reshape(6,6), '\n\n'



def main():
	rospy.init_node('odom_echo', anonymous=True)
	rospy.Subscriber('/odometry/filtered', Odometry, callback)
	rospy.spin()


if __name__ == '__main__':
	main()