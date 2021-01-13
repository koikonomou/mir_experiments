#! /usr/bin/env python

import unittest
import rostest
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry


class OctomapTestCase(unittest.TestCase):
	
	octomap_pointcloud_ok = False

	def octomap_callback(self,data):
		self.octomap_pointcloud_ok = True 
	
	def test_if_octomap_mapping_publishes(self):
		rospy.init_node("octomap_server", anonymous=True)
		rospy.Subscriber("/octomap_point_cloud_centers", PointCloud2, self.octomap_callback)
		# rospy.sleep(0.5)
		counter = 0
		while not rospy.is_shutdown() and counter < 5 and (not self.octomap_pointcloud_ok):
			rospy.sleep(0.5)
			counter += 1

		self.assertTrue(self.octomap_pointcloud_ok)

if __name__ == '__main__':
	rostest.rosrun('mir_experiments', 'test_gazebo_model', OctomapTestCase)
