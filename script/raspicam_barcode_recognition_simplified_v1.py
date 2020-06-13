#!/usr/bin/env python

################################################################################
## {Description}: Code (Bar/QR) Recognition using Raspberry Pi Camera (raspicam)
################################################################################
## Author: Khairul Izwan Bin Kamsani
## Version: {1}.{0}.{0}
## Email: {wansnap@gmail.com}
################################################################################

from __future__ import print_function
from __future__ import division

import sys
import rospy
import cv2
import imutils
import time
import numpy as np
from pyzbar import pyzbar

from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import CameraInfo

from cv_bridge import CvBridge
from cv_bridge import CvBridgeError

class BarcodeRecognition:
	def __init__(self):

		self.bridge = CvBridge()
		self.scanCode = String()
		self.image_received = False
		self.code_received = False
		self.status_received = False

		# Subscribe Image msg
		img_topic = "/raspicam_node_robot/image/compressed"
		self.image_sub = rospy.Subscriber(img_topic, CompressedImage, self.cbImage)

		# Subscribe String msg
		mode_topic = "/scan_mode"
		self.mode_sub = rospy.Subscriber(mode_topic, String, self.cbQRmode)

		# Subscribe String msg
		status_topic = "/scan_status"
		self.status_sub = rospy.Subscriber(status_topic, String, self.cbStatus)

		# Publish String msg
		code_topic = "/scanned_barcode"
		self.code_pub = rospy.Publisher(code_topic, String, queue_size=10)

		rospy.logwarn("BarcodeRecognition Node [ONLINE]...")

		# Allow up to one second to connection
		rospy.sleep(1)

	def cbImage(self, msg):

		# Convert image to OpenCV format
		try:
			cv_image = np.fromstring(msg.data, np.uint8)
			cv_image = cv2.imdecode(cv_image, cv2.IMREAD_COLOR)

			# OPTIONAL -- image-rotate """
			cv_image = imutils.rotate(cv_image, angle=-90)
			cv_image = cv2.flip(cv_image,1)
		except CvBridgeError as e:
			print(e)

		self.image_received = True
		self.image = cv_image

	def cbQRmode(self, msg):

		try:
			mode = msg.data
		except KeyboardInterrupt as e:
			print(e)

		self.code_received = True
		self.typeQR = mode

	def cbStatus(self, msg):

		try:
			scan = msg.data
		except KeyboardInterrupt as e:
			print(e)

		self.status_received = True
		self.status = scan

	def preview(self):

		# Overlay some text onto the image display
		timestr = time.strftime("%Y%m%d-%H%M%S")
		cv2.putText(self.image, timestr, (10, 20), 1, 1, (255, 255, 255), 1, cv2.LINE_AA, False)

		# show the output frame
		cv2.imshow("Frame", self.image)
		cv2.waitKey(1)

	# Get the Scanned Barcode
	def cbBarcode(self):

		if self.image_received:
			# find the barcodes in the frame and decode each of the barcodes
			self.barcodes = pyzbar.decode(self.image)

			if len(self.barcodes) != 0:
				# loop over the detected barcodes
				for self.barcode in self.barcodes:
					# extract the bounding box location of the barcode and 
					# draw the bounding box surrounding the barcode on the 
					# image
					(self.x, self.y, self.w, self.h) = self.barcode.rect
					cv2.rectangle(self.image, (self.x, self.y), 
						(self.x + self.w, self.y + self.h), (0, 0, 255), 2)

					# Allow up to one second to connection
					rospy.sleep(0.005)

					# the barcode data is a bytes object so if we want to 
					# draw it on our output image we need to convert it to 
					# a string first
					self.barcodeData = self.barcode.data.decode("utf-8")
					self.barcodeType = self.barcode.type

					# Publishing
					self.scanCode.data = self.barcodeData
					self.code_pub.publish(self.scanCode)

					if self.code_received:
						cv2.putText(self.image, self.typeQR, (10, 40), 
							1, 1, (255, 255, 255), 1, cv2.LINE_AA, False)
						self.code_received = False

					if self.status_received:
						cv2.putText(self.image, self.status, (10, 60), 
							1, 1, (255, 255, 255), 1, cv2.LINE_AA, False)

					# Refresh the image on the screen
					self.preview()
			else:
				cv2.destroyAllWindows()
				pass
		else:
			rospy.logerr("No images recieved")

if __name__ == '__main__':

	# Initialize
	rospy.init_node("barcode_recognition", anonymous=False)
	barcode = BarcodeRecognition()

	# Camera preview
	while not rospy.is_shutdown():
		barcode.cbBarcode()
