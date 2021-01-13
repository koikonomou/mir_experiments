#! /usr/bin/env python

import unittest
import rostest
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry


class OdomTestCase(unittest.TestCase):
	
	odom_ok = False

	def odom_callback(self,data):
		self.odom_ok = True 
	
	def test_if_odom_publishes(self):
		rospy.init_node("ekf_localization_node", anonymous=True)
		rospy.Subscriber("/odometry/filtered", Odometry, self.odom_callback)
		# rospy.sleep(0.5)
		counter = 0
		while not rospy.is_shutdown() and counter < 5 and (not self.odom_ok):
			rospy.sleep(0.5)
			counter += 1

		self.assertTrue(self.odom_ok)

if __name__ == '__main__':
	rostest.rosrun('mir_experiments', 'test_odom', OdomTestCase)
