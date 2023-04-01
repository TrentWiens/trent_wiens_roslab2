#!/usr/bin/env python

import rospy
import numpy as np

import tf
import tf2_ros

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import geometry_msgs


class Safety(object):

	def __init__(self):
	
		self.speed = .5
		self.scan = rospy.Subscriber("scan", LaserScan, self.scan_callback)
		self.tfBuffer = tf2_ros.Buffer()
		self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
		
		self.pPubX = rospy.Publisher("pX", Float32, queue_size = 10)
		self.pPubY = rospy.Publisher("pY", Float32, queue_size = 10)
		self.pPrimePubX = rospy.Publisher("pPrimeX", Float32, queue_size = 10)
		self.pPrimePubY = rospy.Publisher("pPrimeY", Float32, queue_size = 10)
		self.TTCPub = rospy.Publisher("TTC", Float32, queue_size = 10)

	def scan_callback(self, scan_msg):
		
		minAngle = scan_msg.angle_min
		incAngle = scan_msg.angle_increment
		ranges = scan_msg.ranges
		speed = self.speed
		transf = self.tfBuffer.lookup_transform("laser", "base_link", rospy.Time())


		smallestRange = np.amin(ranges)
		i = ranges.index(smallestRange)
		angle = minAngle + incAngle * i
		xl = smallestRange * np.sin(angle)
		yl = smallestRange * np.cos(angle) 
		zl = 0
		
		laserPose = geometry_msgs.msg.PoseStamped()
		laserPose.header.frame_id = "/laser"
		laserPose.pose.position.x = xl
		laserPose.pose.position.y = yl
		laserPose.pose.position.z = zl
			
		xb = xl + transf.transform.translation.x
		yb = yl + transf.transform.translation.y
		zb = zl + transf.transform.translation.z
		
		rb = np.sqrt(xb**2 + yb**2 + zb**2)
		
		basePose = geometry_msgs.msg.PoseStamped()
		basePose.header.frame_id = "/base_frame"
		basePose.pose.position.x = xb
		basePose.pose.position.y = yb
		basePose.pose.position.z = zb
		
		rDot = speed * np.cos(angle)	
		rDot = np.maximum(rDot, 0.0)

			
		if not rDot == 0:
			TTC = rb/rDot
		else:
			TTC = 1000
			
		
		self.pPubX.publish(laserPose.pose.position.x)
		self.pPubY.publish(laserPose.pose.position.y)
		self.pPrimePubX.publish(basePose.pose.position.x)
		self.pPrimePubY.publish(basePose.pose.position.y)
		
		self.TTCPub.publish(TTC)
		
				
def main():
	rospy.init_node('safety_node')
	rospy.Rate(1000)
	sn = Safety()
	rospy.spin()
if __name__ == '__main__':
	main()
