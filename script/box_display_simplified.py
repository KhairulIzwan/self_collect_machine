#!/usr/bin/env python3

################################################################################
## {Description}: Validate the QR/Bar Code (for USB type camera)
################################################################################
## Author: Khairul Izwan Bin Kamsani
## Version: {1}.{0}.{0}
## Email: {wansnap@gmail.com}
################################################################################

import sys
import rospy

from std_msgs.msg import String
from std_msgs.msg import Int32
from std_msgs.msg import Bool
from self_collect_machine.msg import boxStatus

import RPi.GPIO as GPIO
from time import sleep, strftime
from datetime import datetime

from luma.core.interface.serial import spi, noop
from luma.core.render import canvas
from luma.core.virtual import viewport
from luma.led_matrix.device import max7219
from luma.core.legacy import text, show_message
from luma.core.legacy.font import proportional, CP437_FONT, LCD_FONT

class BoxIDDisplay:
	def __init__(self):
		self.serial = spi(port=0, device=0, gpio=noop())
		self.device = max7219(self.serial, width=32, height=8, block_orientation=-90)
		self.device.contrast(5)
		self.virtual = viewport(self.device, width=32, height=16)

		rospy.on_shutdown(self.shutdown)

		self.sensor_received = False
		self.code_received = False
		self.box_received = False

		# Subscribe Int32 msg
		self.sensor_topic = "/boxID_activation"
		self.sensor_sub = rospy.Subscriber(self.sensor_topic, Int32, self.cbID)

		# Subscribe Int32 msg
		self.box_topic = "/box_position"
		self.box_sub = rospy.Subscriber(self.box_topic, Int32, self.cbBox)

		# Subscribe String msg
		self.mode_topic = "/scan_mode"
		self.mode_sub = rospy.Subscriber(self.mode_topic, String, self.cbQRmode)

	def cbID(self, msg):

		try:
			self.sensor = msg.data
		except KeyboardInterrupt as e:
			print(e)

		self.sensor_received = True

	def cbQRmode(self, msg):

		try:
			self.mode = msg.data
		except KeyboardInterrupt as e:
			print(e)

		self.update_display()

	def cbBox(self, msg):

		try:
			self.box = msg.data
		except KeyboardInterrupt as e:
			print(e)

		self.box_received = True

	def shutdown(self):
		try:
			rospy.logwarn("BoxIDDisplay node [OFFLINE]")
		finally:
			pass

	def update_display(self):

		if self.mode == "customer":
			with canvas(self.virtual) as draw:
				text(draw, (1, 1), "CUST", fill="white", 
					font=proportional(CP437_FONT))
			rospy.sleep(1)
			if self.sensor_received:
				with canvas(self.virtual) as draw:
					text(draw, (1, 1), "{}".format(self.sensor), 
						fill="white", font=proportional(CP437_FONT))
				rospy.sleep(5)
				self.sensor_received = False
			else:
				with canvas(self.virtual) as draw:
					text(draw, (1, 1), "N/A", 
						fill="white", font=proportional(CP437_FONT))
				rospy.sleep(5)
				self.mode = "N/A"
		elif self.mode == "store":
			with canvas(self.virtual) as draw:
				text(draw, (1, 1), "STOR", fill="white", 
					font=proportional(CP437_FONT))
			rospy.sleep(1)
			if self.box_received:
				with canvas(self.virtual) as draw:
					text(draw, (1, 1), "{}".format(self.box), 
						fill="white", font=proportional(CP437_FONT))
				rospy.sleep(5)
				self.box_received = False
			else:
				with canvas(self.virtual) as draw:
					text(draw, (1, 1), "N/A", 
						fill="white", font=proportional(CP437_FONT))
				rospy.sleep(5)
				self.mode = "N/A"
		else:
			with canvas(self.virtual) as draw:
				text(draw, (1, 1), "SCAN", fill="white", font=proportional(CP437_FONT))

def main(args):
	display = BoxIDDisplay()
	rospy.init_node("boxID_display", anonymous=False)

	try:
		rospy.spin()
	except KeyboardInterrupt as e:
		print(e)

if __name__ == '__main__':
	main(sys.argv)
