#!/usr/bin/env python

################################################################################
## {Description}: Validate the QR/Bar Code (for USB type camera)
################################################################################
## Author: Khairul Izwan Bin Kamsani
## Version: {1}.{0}.{0}
## Email: {wansnap@gmail.com}
################################################################################

from __future__ import print_function
from __future__ import division

import sys
import rospy

from std_msgs.msg import String
from std_msgs.msg import Int32
from self_collect_machine.msg import boxStatus

import csv
import datetime
import cv2

import smbus

i2c = smbus.SMBus(1)
I2C_ADD = 0x09 # Arduino I2C address

class BoxIDValidate_node:
	def __init__(self):

		self.boxState = boxStatus()

		# Initializing your ROS Node
		rospy.init_node('box_validity', anonymous=False)

		# Publish boxStatus msg
		self.boxStatus_pub = rospy.Publisher("/box_available", boxStatus, queue_size=10)

		self.getBoxState()

	def getBoxState(self):

		while not rospy.is_shutdown():
			# Read
			I2Cdata = readI2C()
			if I2Cdata != prevI2CData:
				prevI2CData = I2Cdata
				if I2Cdata == 1:
					boxState = [1, 1, 1]
		
				elif I2Cdata == 2:
					boxState = [1, 1, 0]

				elif I2Cdata == 3:
					boxState = [1, 0, 1]

				elif I2Cdata == 4:
					boxState = [1, 0, 0]

				elif I2Cdata == 5:
					boxState = [0, 1, 1]

				elif I2Cdata == 6:
					boxState = [0, 1, 0]

				elif I2Cdata == 7:
					boxState = [0, 0, 1]

				elif I2Cdata == 8:
					boxState = [0, 0, 0]

			self.boxState.data = boxState
			self.boxStatus_pub.publish(self.boxState)

def readI2C():
	inData = i2c.read_byte(I2C_ADD)
	return inData

def main(args):

	vn = BoxIDValidate_node()

	try:
		rospy.spin()
	except KeyboardInterrupt:
		rospy.loginfo("[INFO] BoxIDValidate_node [OFFLINE]...")

if __name__ == '__main__':
	main(sys.argv)
