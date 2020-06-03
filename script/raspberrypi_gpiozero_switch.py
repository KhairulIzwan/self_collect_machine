from gpiozero import Button
import smbus
from time import sleep

SW1 = Button(21)

while True:
	if SW1.is_pressed:
		print("[INFO] SW1 Pressed!")
	else:
		print("[INFO] SW1 Not Pressed!")
	sleep(0.1)

GPIO.cleanup()
