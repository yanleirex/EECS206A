#!/usr/bin/env python

import tf2_ros
import rospy
import math
import tf
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped 

if __name__ == '__main__':
	rospy.init_node('listener')
	tfBuffer = tf2_ros.Buffer()
	tfListener = tf2_ros.TransformListener(tfBuffer)
	while not rospy.is_shutdown():
		try:
			trans = tfBuffer.lookup_transform('left_hand', 'base', rospy.Time())
			print(trans.transform)
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue
