from gpiozero import Servo
import pygame
import RPi.GPIO as GPIO
import os
import time
import pigpio
import numpy as np
import os
os.system('sudo pigpiod')

from gpiozero.pins.pigpio import PiGPIOFactory
factory = PiGPIOFactory()

hw_pwm = 12
servo = Servo(hw_pwm, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000, pin_factory=factory)
pi = pigpio.pi()
pi.set_mode(hw_pwm, pigpio.OUTPUT)


#from picamera import PiCamera
from datetime import datetime
import board
import busio

i2c = busio.I2C(board.SCL, board.SDA)

import adafruit_ads1x15.ads1015 as ADS

from adafruit_ads1x15.analog_in import AnalogIn

ads = ADS.ADS1015(i2c)
chan = AnalogIn(ads, ADS.P0)

print(chan.value, chan.voltage)
volt = ((chan.voltage) * 37500 / 7500)
print("volt = {}".format((chan.voltage)*37500 / 7500))

input_voltage = volt

screen = pygame.display.set_mode([240, 160])

#camera = PiCamera()
#camera.resolution = (1280, 720)
#camera.framerate = (25)

steps = 1500
pwm1 = 26 #pwm forward
pwm2 = 19 #pwm backward
logic1 = 5 #forward
logic2 = 6 #backward
pwm_freq = 1000 #pwm freq set at 2 KHz #duty cycle is in percentage
rec = 29
vout_default = 6
vout_max = 8

dc_default = 100 * vout_default / input_voltage
dc_boost = 100 * vout_max / input_voltage

#record = 0

#set GPIO num
GPIO.setmode(GPIO.BCM)
GPIO.setup(pwm1, GPIO.OUT)
GPIO.setup(pwm2, GPIO.OUT)
GPIO.setup(logic1, GPIO.OUT)
GPIO.setup(logic2, GPIO.OUT)
#GPIO.setup(rec, GPIO.OUT)

#setup PWM control
speedforward = GPIO.PWM(pwm1, pwm_freq)
speedbackward = GPIO.PWM(pwm2, pwm_freq)
speedforward.start(0)  #duty cycle is in percentage
speedbackward.start(0) #duty cycle is in percentage


try:
	while True:
		for event in pygame.event.get():
			if event.type == pygame.KEYDOWN:
				if event.key == pygame.K_q:
					pygame.quit()
				#elif event.key == pygame.K_s:
				#	os.system('sudo shutdown now')
				#elif event.key == pygame.K_r:
				#	if record == 0:
				#		record = 1
				#		moment = datetime.now()
				#		GPIO.output(rec, False)
				#		camera.start_recording('/home/pi/Vid_%02d')
				#elif event.key == pygame.K_t:
				#	if record == 1:
				#		record = 0
				#		GPIO.output(rec, True)
				#		camera.stop_recording()
				elif event.key == pygame.K_d:
					speedforward.start(dc_default)
					speedbackward.start(0)
					speedforward.ChangeDutyCycle(dc_default)
					speedbackward.ChangeDutyCycle(0)
				elif event.key == pygame.K_b:
					speedforward.start(dc_boost)
					speedbackward.start(0)
					speedforward.ChangeDutyCycle(dc_boost)
					speedbackward.ChangeDutyCycle(0)
				if event.key == pygame.K_LEFT:
					for steps in range (steps, 500, -10):
						#servo.value = steps;
						pi.set_servo_pulsewidth(hw_pwm, steps)
				if event.key == pygame.K_RIGHT:
					for steps in range (steps, 2500, 10):
						#servo.value = steps;
						pi.set_servo_pulsewidth(hw_pwm, steps)
				elif event.key == pygame.K_UP:
					GPIO.output(logic1, True)
					GPIO.output(logic2, False)
				elif event.key == pygame.K_DOWN:
					GPIO.output(logic1, False)
					GPIO.output(logic2, True)
				elif event.key == pygame.K_1:
					speedforward.start(75)
					speedbackward.start(0)
					speedforward.ChangeDutyCycle(75) #duty cycle is in percentage
					speedbackward.ChangeDutyCycle(0) #duty cycle is in percentage
				elif event.key == pygame.K_2:
					speedforward.start(85)
					servo.value = None;
					speedbackward.start(0)
					speedforward.ChangeDutyCycle(85) #duty cycle is in percentage
					speedbackward.ChangeDutyCycle(0) #duty cycle is in percentage
				elif event.key == pygame.K_3:
					speedforward.start(100)
					speedbackward.start(0)
					speedforward.ChangeDutyCycle(100) #duty cycle is in percentage
					speedbackward.ChangeDutyCycle(0) #duty cycle is in percentage
			if event.type == pygame.KEYUP:
				GPIO.output(logic1, False)
				GPIO.output(logic2, False)
				speedforward.stop()
				speedbackward.stop()
				#servo.value = None;
				pi.set_servo_pulsewidth(hw_pwm, 0)

finally:
	GPIO.cleanup()
	#servo.close()
