import pygame
from pygame.locals import *
import sys

pygame.init()
display = pygame.display.set_mode((300, 300))

import time
from gpiozero import Servo
import RPi.GPIO as GPIO
import pigpio
import os
os.system('sudo pigpiod')

from gpiozero.pins.pigpio import PiGPIOFactory
factory = PiGPIOFactory()

hw_pwm = 12
servo = Servo(hw_pwm, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000, pin_factory=factory)
pi = pigpio.pi()
pi.set_mode(hw_pwm, pigpio.OUTPUT)

from datetime import datetime
import board
import busio

i2c = busio.I2C(board.SCL, board.SDA)

import adafruit_ads1x15.ads1015 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

ads = ADS.ADS1015(i2c)
chan = AnalogIn(ads, ADS.P0)
print(chan.voltage, chan.voltage)
volt = ((chan.voltage) * 37500 / 7500)
print("volt = {}".format((chan.voltage)*37500 / 7500))
input_voltage = volt

steps = 1500
pwm1 = 26
pwm2 = 19
pwm_freq = 1000
vout_default = 6
vout_max = 8

dc_default = 100 * vout_default / input_voltage
dc_boos = 100 * vout_max / input_voltage

GPIO.setmode(GPIO.BCM)
GPIO.setup(pwm1, GPIO.OUT)
GPIO.setup(pwm2, GPIO.OUT)

speedforward = GPIO.PWM(pwm1, pwm_freq)
speedbackward = GPIO.PWM(pwm2, pwm_freq) 
speedforward.start(0)
speedbackward.start(0)


while 1:
	pressed_keys = pygame.key.get_pressed()

	if pressed_keys[K_q]:
		print("key q has been pressed")
		pygame.quit()
	if pressed_keys[K_d]:
		print("key d has been pressed")
		#time.sleep(0.05)
		speedforward.start(dc_default)
		speedbackward.start(0)
		speedforward.ChangeDutyCycle(dc_default)
		speedbackward.ChangeDutyCycle(0)
	if pressed_keys[K_d] and pressed_keys[K_LEFT]:
		print("keys d and left has been pressed")
		#time.sleep(0.05)
		speedforward.start(dc_default)
		speedbackward.start(0)
		speedforward.ChangeDutyCycle(dc_default)
		speedbackward.ChangeDutyCycle(0)
		for steps in range (steps, 500, -10):
			pi.set_servo_pulsewidth(hw_pwm, steps)
	elif pressed_keys[K_d] and pressed_keys[K_RIGHT]:
		print("keys d and right has been pressed")
		#time.sleep(0.05)
		speedforward.start(dc_default)
		speedbackward.start(0)
		speedforward.ChangeDutyCycle(dc_default)
		speedbackward.ChangeDutyCycle(0)
		for steps in range (steps, 2500, 10):
			pi.set_servo_pulsewidth(hw_pwm, steps)
	else: #stop the motors if no button is pressed
		speedforward.stop()
		speedbackward.stop()
		pi.set_servo_pulsewidth(hw_pwm, 0)


	pygame.event.pump()
