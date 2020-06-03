from gpiozero import Button, LED
import smbus
from time import sleep

LED1 = LED(21)
LED2 = LED(20)

i2c = smbus.SMBus(1)
I2C_ADD = 0x09 # Arduino I2C address

prevI2CData = 0

def readI2C():
	inData = i2c.read_byte(I2C_ADD)
	return inData

def writeI2C(data):
	i2c.write_byte(I2C_ADD, data)

try:
	while True:

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

			print(boxState)

except KeyboardInterrupt:
	pass
