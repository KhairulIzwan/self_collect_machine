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

class BoxIDDisplay_node:
	def __init__(self):
		self.serial = spi(port=0, device=0, gpio=noop())
		self.device = max7219(self.serial, width=32, height=8, block_orientation=-90)
		self.device.contrast(5)
		self.virtual = viewport(self.device, width=32, height=16)

		self.sensor_received = False
		self.code_received = False
		self.box_received = False

		# Subscribe Int32 msg
		sensor_topic = "/boxID_activation"
		self.sensor_sub = rospy.Subscriber(sensor_topic, Int32, self.cbID)

		# Subscribe Int32 msg
		box_topic = "/box_position"
		self.box_sub = rospy.Subscriber(box_topic, Int32, self.cbBox)

		# Subscribe String msg
		mode_topic = "/scan_mode"
		self.mode_sub = rospy.Subscriber(mode_topic, String, self.cbQRmode)

	def cbID(self, msg):

		try:
			sensor = msg.data
		except KeyboardInterrupt as e:
			print(e)

		self.sensor_received = True
		self.sensor_value = sensor

	def cbQRmode(self, msg):

		try:
			mode = msg.data
		except KeyboardInterrupt as e:
			print(e)

		self.code_received = True
		self.typeQR = mode

	def cbBox(self, msg):

		try:
			box = msg.data
		except KeyboardInterrupt as e:
			print(e)

		self.box_received = True
		self.boxuse = box

	def update_display(self):

		if self.code_received:
			if self.typeQR == "customer":
				if self.sensor_received:
					with canvas(self.virtual) as draw:
						text(draw, (1, 1), "{}".format(self.sensor_value), 
							fill="white", font=proportional(CP437_FONT))
					rospy.sleep(5)
					self.code_received = False
				else:
					with canvas(self.virtual) as draw:
						text(draw, (1, 1), "SCAN", fill="white", font=proportional(CP437_FONT))
			else:
				#TODO
				pass

			if self.typeQR == "store":
				if self.box_received:
					with canvas(self.virtual) as draw:
						text(draw, (1, 1), "{}".format(self.boxuse), 
							fill="white", font=proportional(CP437_FONT))
					rospy.sleep(5)
					self.code_received = False
				else:
					with canvas(self.virtual) as draw:
						text(draw, (1, 1), "SCAN", fill="white", font=proportional(CP437_FONT))
			else:
				#TODO
				pass
		else:
			with canvas(self.virtual) as draw:
				text(draw, (1, 1), "SCAN", fill="white", font=proportional(CP437_FONT))
			pass

if __name__ == '__main__':

	# Initialize
	rospy.init_node('BoxIDDisplay_node', anonymous=False)
	led = BoxIDDisplay_node()

	# Take a photo
	while not rospy.is_shutdown():
		led.update_display()

	# Sleep to give the last log messages time to be sent
	rospy.sleep(1)
