from gpiozero import Button
import smbus
from time import sleep

SW1 = Button(21)
SW = Button(20)

i2c = smbus.SMBus(1)
I2C_ADD = 0x09 # Arduino I2C address

def writeI2C(data):
	i2c.write_byte(I2C_ADD, data)

try:
	while True:
		if SW1.is_pressed:
			writeI2C(1)
			print("[INFO] SW1 Pressed!")
		else:
			writeI2C(2)
			print("[INFO] SW1 Not Pressed!")
		sleep(0.1)

except KeyboardInterrupt:
	GPIO.cleanup()
