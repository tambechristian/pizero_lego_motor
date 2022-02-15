import RPi.GPIO as GPIO
from time import sleep

GPIO.setmode(GPIO.BCM)

C1 = 12
C2 = 13

moveFactor = 14.285714286

GPIO.setup(C1, GPIO.OUT)
GPIO.setup(C2, GPIO.OUT)

pwm1 = GPIO.PWM(C1, 1200)
pwm2 = GPIO.PWM(C2, 1200)

def setPosition(position):
	pwm1.stop()
	pwm2.stop()
	i = round(position * moveFactor, 2)

	if position >= 0:
		print ('Position: ' + str(position) + 'at a Duty Cycle of ' + str(i))
		pwm1.start(i)
	else:
		print ('Position: ' + str(position) + 'at a Duty Cycle of' + str(i*-1))
		pwm2.start(i*-1)

try:
	setPosition(-7)
	sleep(2)
	setPosition(7)
	sleep(1)
except:
	pass

pwm1.stop()
pwm2.stop()
GPIO.cleanup()

