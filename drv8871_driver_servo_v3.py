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

hw_pwm1 = 12
hw_pwm2 = 13
pwm3 = 26
pwm4 = 19
hw_freq = 1200

pi1 = pigpio.pi()
pi2 = pigpio.pi()
pi3 = pigpio.pi()
pi4 = pigpio.pi()

pi1.set_mode(hw_pwm1, pigpio.OUTPUT)
pi2.set_mode(hw_pwm2, pigpio.OUTPUT)
pi3.set_mode(pwm3, pigpio.OUTPUT)
pi4.set_mode(pwm4, pigpio.OUTPUT)

pi3.set_PWM_range(pwm3, 3000) # setting the pwm range
pi4.set_PWM_range(pwm4, 3000) # setting the pwm range

pi3.set_PWM_frequency(pwm3, 1200)
print("hw pwm3 freq: {}".format(pi3.get_PWM_frequency(pwm3)))
pi4.set_PWM_frequency(pwm4, 1200)
print("hw pwm4 freq: {}".format(pi4.get_PWM_frequency(pwm4)))

dc1 = 0
dc2 = 0
dc3 = 0
dc4 = 0

from datetime import datetime
import board
import busio

i2c = busio.I2C(board.SCL, board.SDA)

import adafruit_ads1x15.ads1015 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

ads = ADS.ADS1015(i2c)
chan = AnalogIn(ads, ADS.P0)
#print(chan.voltage, chan.voltage)
batt_volt = ((chan.voltage) * 37500 / 7500)
print("batt_volt = {}".format((chan.voltage)*37500 / 7500))

if (batt_volt < 9.00):
	print("Low battery!!!")

volt_scale_factor = batt_volt / 12.00
lmotors_max_volt = batt_volt / volt_scale_factor
print("lmotors_max_volt = {}".format(lmotors_max_volt))

lmotors_max_volt_DutyCycle = (100 * lmotors_max_volt )/ batt_volt

dc_default = 9.00 * lmotors_max_volt_DutyCycle / lmotors_max_volt
#dc_default = 20
print("default duty cycle is: {}".format(dc_default))

#dc_default = 100 * (lmotors_max_volt - 3) / batt_volt

default_dutycycle = 2250
dc_boost = 100 * lmotors_max_volt / batt_volt

pwm_freq = 1200
vout_default = 9
vout_max = 12

led_ain1 = 21
led_ain2 = 20
led_en = 16
GPIO.setmode(GPIO.BCM)
#GPIO.setup(pwm1, GPIO.OUT)
#GPIO.setup(pwm2, GPIO.OUT)
GPIO.setup(led_ain1, GPIO.OUT)
GPIO.setup(led_ain2, GPIO.OUT)
GPIO.setup(led_en, GPIO.OUT)

GPIO.output(led_en, GPIO.HIGH)

#speedforward = GPIO.PWM(pwm1, pwm_freq)
#speedbackward = GPIO.PWM(pwm2, pwm_freq) 
#speedforward.start(0)
#speedbackward.start(0)


while 1:
	pressed_keys = pygame.key.get_pressed()

	if pressed_keys[K_q]:
		print("key q (quit) has been pressed")
		pi1.hardware_PWM(hw_pwm1, 1000, 0)
		pi2.hardware_PWM(hw_pwm2, 1000, 0)
		pygame.quit()
		exit()
	if pressed_keys[K_i]:
		time.sleep(0.5)
		print("key i (info) has been pressed")
		batt_volt = ((chan.voltage) * 37500 / 7500)
		dc_default = 100 * vout_default / batt_volt
		lmotors_max_volt = batt_volt / volt_scale_factor
		print("dc default: {}".format(dc_default))
		print("battery voltage: {}".format(batt_volt))
		print("lmotors_max_volt: {}".format(lmotors_max_volt))
		if (batt_volt) <9.0:
			print("low battery!!!")
	if pressed_keys[K_l]:
		print("key l (light) has been pressed")
		time.sleep(0.05)
		GPIO.output(led_ain1, GPIO.HIGH)
		GPIO.output(led_ain2, GPIO.HIGH)
	if pressed_keys[K_o]:
		print("keys o (light off) has been pressed")
		time.sleep(0.05)
		GPIO.output(led_ain1, GPIO.LOW)
		GPIO.output(led_ain2, GPIO.LOW)
	if pressed_keys[K_d]:
		#print("key d (drive) has been pressed")
		#time.sleep(0.05)
		#speedforward.start(dc_default)
		#speedbackward.start(0)
		#speedforward.ChangeDutyCycle(dc_default)
		#speedbackward.ChangeDutyCycle(0)
		#pi3.set_PWM_dutycycle(pwm3, dc_default)
		#pi4.set_PWM_dutycycle(pwm4, 0)
		pi4.set_PWM_dutycycle(pwm4, 0)
		dc3 = 1500
		if (dc3 < default_dutycycle):
			dc3 = dc3 + 10
			pi3.set_PWM_dutycycle(pwm3, dc3)
		else:
			pi3.set_PWM_dutycycle(pwm3, dc3)

	if pressed_keys[K_r]:
		#print("key r (reverse) has been pressed")
		#speedforward.start(0)
		#speedbackward.start(dc_default)
		#speedforward.ChangeDutyCycle(0)
		#speedbackward.ChangeDutyCycle(dc_default)
		#pi4.set_PWM_dutycycle(pwm4, dc_default)
		#pi3.set_PWM_dutycycle(pwm3, 0)
		pi3.set_PWM_dutycycle(pwm3, 0)
		dc4 = 1500
		if (dc4 < default_dutycycle):
			dc4 = dc4 + 10
			pi4.set_PWM_dutycycle(pwm4, dc4)
		else:
			pi4.set_PWM_dutycycle(pwm4, dc4)

	if pressed_keys[K_d] and pressed_keys[K_LEFT]:
		#print("keys d and left has been pressed")
		#speedforward.start(dc_default)
		#speedbackward.start(0)
		#speedforward.ChangeDutyCycle(dc_default)
		#speedbackward.ChangeDutyCycle(0)
		if (dc2 <1000000):
			dc2 = dc2 + 500
			pi2.hardware_PWM(hw_pwm2, 1000, dc2)
		else:
			pi2.hardware_PWM(hw_pwm2, 1000, dc2)

	elif pressed_keys[K_d] and pressed_keys[K_RIGHT]:
		#print("keys d and right has been pressed")
		#time.sleep(0.05)
		#speedforward.start(dc_default)
		#speedbackward.start(0)
		#speedforward.ChangeDutyCycle(dc_default)
		#speedbackward.ChangeDutyCycle(0)
		if (dc1 <1000000):
			dc1 = dc1 + 500
			pi1.hardware_PWM(hw_pwm1, 1000, dc1)
		else:
			pi1.hardware_PWM(hw_pwm1, 1000, dc1)

	elif pressed_keys[K_RIGHT]:
		if (dc1 <1000000):
			dc1 = dc1 + 500
			pi1.hardware_PWM(hw_pwm1, 1000, dc1)
		else:
			pi1.hardware_PWM(hw_pwm1, 1000, dc1)

	elif pressed_keys[K_LEFT]:
		if (dc2 <1000000):
			dc2 = dc2 + 500
			pi2.hardware_PWM(hw_pwm2, 1000, dc2)
		else:
			pi2.hardware_PWM(hw_pwm2, 1000, dc2)
	else: #stop the motors if no button is pressed
		dc1 = 0
		dc2 = 0
		pi1.hardware_PWM(hw_pwm1, 1000, dc1)
		pi2.hardware_PWM(hw_pwm2, 1000, dc2)
		pi3.set_PWM_dutycycle(pwm3, dc3)
		pi4.set_PWM_dutycycle(pwm4, dc4) 
		if (dc3 > 0):
			dc3 = dc3 - 1
			pi3.set_PWM_dutycycle(pwm3, dc3)
		#else:
		#	pi3.set_PWM_dutycycle(pwm3, dc3) 
		if (dc4 > 0):
			dc4 = dc4 - 1 
			pi4.set_PWM_dutycycle(pwm4, dc4) 
		#else:
		#	pi4.set_PWM_dutycycle(pwm4, dc4)  

	pygame.event.pump()

