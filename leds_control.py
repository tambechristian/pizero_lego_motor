import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setup(23, GPIO.OUT)
GPIO.setup(24, GPIO.OUT)
GPIO.setup(25, GPIO.OUT)

for i in range (10):
	time.sleep(0.5)
	GPIO.output(23, GPIO.HIGH)
	time.sleep(0.5)
	GPIO.output(23, GPIO.LOW)
	time.sleep(0.5)
	GPIO.output(24, GPIO.HIGH)
	time.sleep(0.5)
	GPIO.output(24, GPIO.LOW)
	time.sleep(0.5)
	GPIO.output(25, GPIO.HIGH)
	time.sleep(0.5)
	GPIO.output(25, GPIO.LOW)


