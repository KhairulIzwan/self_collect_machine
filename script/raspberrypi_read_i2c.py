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

try:
	while True:
		I2Cdata = readI2C()
		if I2Cdata != prevI2CData:
			prevI2CData = I2Cdata
			if I2Cdata == 1:
				LED1.on()
				LED2.off()
		
			elif I2Cdata == 2:
				LED1.off()
				LED2.on()

			sleep(0.1)

except KeyboardInterrupt:
	GPIO.cleanup()
