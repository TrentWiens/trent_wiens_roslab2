#!/usr/bin/env python

import rospy
import numpy as np
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from std_msgs.msg import Bool, Float32


class Safety(object):

	def __init__(self):
	
		self.TTCSub = rospy.Subscriber("TTC", Float32, self.TTC_callback)
		self.brakePub = rospy.Publisher("brake", AckermannDriveStamped, queue_size = 10)
		self.brakeBoolPub = rospy.Publisher("brake_bool", Bool, queue_size = 10)

	def TTC_callback(self, TTC_msg):
	
		TTCThreshold = 4
		TTC = TTC_msg.data
		
		if TTC < TTCThreshold:
			brake_bool = True
			self.brakeBoolPub.publish(brake_bool)
			ackMsg = AckermannDriveStamped()
			ackMsg.drive.speed = 0
			self.brakePub.publish(ackMsg)
		else: 
			brake_bool = False
			self.brakeBoolPub.publish(brake_bool)
				
def main():
	rospy.init_node('safety_node')
	rospy.Rate(1000)
	sn = Safety()
	rospy.spin()
if __name__ == '__main__':
	main()
