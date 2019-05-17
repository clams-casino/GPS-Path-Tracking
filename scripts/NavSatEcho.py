#!/usr/bin/env python

import rospy
import numpy as np

from sensor_msgs.msg import NavSatFix


def callback(data):
	rospy.loginfo("latitude: %s \nlongitude: %s" % (data.latitude, data.longitude))
	print np.array([1,2,3])

def main():

	rospy.init_node('navsat_echo', anonymous=True)
	rospy.Subscriber('/navsat/fix', NavSatFix, callback)
	rospy.spin()


if __name__ == '__main__':
	main()