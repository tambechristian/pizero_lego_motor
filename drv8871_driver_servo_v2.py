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
hw_freq = 1200

pi1 = pigpio.pi()
pi2 = pigpio.pi()

pi1.set_mode(hw_pwm1, pigpio.OUTPUT)
pi2.set_mode(hw_pwm2, pigpio.OUTPUT)

pi1.set_PWM_range(hw_pwm1, 255)
pi2.set_PWM_range(hw_pwm2, 255)

pi1.set_PWM_frequency(hw_pwm1, 1200)
print("hw pwm1 freq: {}".format(pi1.get_PWM_frequency(hw_pwm1)))
pi2.set_PWM_frequency(hw_pwm2, 1200)
print("hw pwm2 freq: {}".format(pi2.get_PWM_frequency(hw_pwm2)))


max_dc2 = 255
max_dc1 = 255
position1 = 0
position2 = 0
current_dc1 = 0
current_dc2 = 0
curr_pulse1 = 1500
curr_pulse2 = 1500
dc1 = 0
dc2 = 0
#servo1 = Servo(hw_pwm1, min_pulse_width=1/1000, max_pulse_width=2/1000, pin_factory=factory)
#servo2 = Servo(hw_pwm2, min_pulse_width=1/1000, max_pulse_width=2/1000, pin_factory = factory)
#servo2 = Servo(hw_pwm2, pin_factory = factory)


from datetime import datetime
import board
import busio

i2c = busio.I2C(board.SCL, board.SDA)

import adafruit_ads1x15.ads1015 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

ads = ADS.ADS1015(i2c)
chan = AnalogIn(ads, ADS.P0)
print(chan.voltage, chan.voltage)
batt_volt = ((chan.voltage) * 37500 / 7500)
print("batt_volt = {}".format((chan.voltage)*37500 / 7500))

volt_scale_factor = batt_volt / 12.00
lmotors_max_volt = batt_volt / volt_scale_factor
print("lmotors_max_volt = {}".format(lmotors_max_volt))

#if (volt

dc_default = 100 * (lmotors_max_volt - 3) / batt_volt
dc_boost = 100 * lmotors_max_volt / batt_volt

pwm1 = 26
pwm2 = 19
pwm_freq = 1200
vout_default = 9
vout_max = 12

led_ain1 = 21
led_ain2 = 20
led_en = 16
GPIO.setmode(GPIO.BCM)
GPIO.setup(pwm1, GPIO.OUT)
GPIO.setup(pwm2, GPIO.OUT)
GPIO.setup(led_ain1, GPIO.OUT)
GPIO.setup(led_ain2, GPIO.OUT)
GPIO.setup(led_en, GPIO.OUT)

GPIO.output(led_en, GPIO.HIGH)

speedforward = GPIO.PWM(pwm1, pwm_freq)
speedbackward = GPIO.PWM(pwm2, pwm_freq) 
speedforward.start(0)
speedbackward.start(0)


while 1:
	current_dc1 = position1
	current_dc2 = position2
	pressed_keys = pygame.key.get_pressed()

	if pressed_keys[K_q]:
		print("key q (quit) has been pressed")
		pi1.set_PWM_dutycycle(hw_pwm1, 0)
		pi2.set_PWM_dutycycle(hw_pwm2, 0)
		time.sleep(0.5)
		pygame.quit()
	if pressed_keys[K_i]:
		time.sleep(0.5)
		print("key i (info) has been pressed")
		batt_volt = ((chan.voltage) * 37500 / 7500)
		dc_default = 100 * vout_default / batt_volt
		lmotors_max_volt = batt_volt / volt_scale_factor
		print("dc default: {}".format(dc_default))
		print("battery voltage: {}".format(batt_volt))
		print("lmotors_max_volt: {}".format(lmotors_max_volt))
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
		speedforward.start(dc_default)
		#print("dc default: {}on drive".format(dc_default))
		speedbackward.start(0)
		speedforward.ChangeDutyCycle(dc_default)
		speedbackward.ChangeDutyCycle(0)
	if pressed_keys[K_r]:
		print("key r (reverse) has been pressed")
		speedforward.start(0)
		speedbackward.start(dc_default)
		speedforward.ChangeDutyCycle(0)
		speedbackward.ChangeDutyCycle(dc_default)
	if pressed_keys[K_d] and pressed_keys[K_LEFT]:
		print("keys d and left has been pressed")
		#time.sleep(0.05)
		speedforward.start(dc_default)
		speedbackward.start(0)
		#speedforward.ChangeDutyCycle(dc_default)
		#speedbackward.ChangeDutyCycle(0)
		for steps in range (steps, 500, -10):
			pi1.set_servo_pulsewidth(hw_pwm1, steps)
	elif pressed_keys[K_d] and pressed_keys[K_RIGHT]:
		print("keys d and right has been pressed")
		#time.sleep(0.05)
		speedforward.start(dc_default)
		speedbackward.start(0)
		#speedforward.ChangeDutyCycle(dc_default)
		#speedbackward.ChangeDutyCycle(0)
		for steps in range (steps, 2500, 10):
			pi1.set_servo_pulsewidth(hw_pwm1, steps)
	elif pressed_keys[K_RIGHT]:
		print("key RIGHT has been pressed")
		if (dc1 <255):
			dc1 = dc1 +1
			pi1.set_PWM_dutycycle(hw_pwm1, dc1)
			if (dc2 > 0):
				dc2 = dc2 -1
				pi2.set_PWM_dutycycle(hw_pwm2, dc2)
			else:
				print("maximum right steer reached")
				pi2.set_PWM_dutycycle(hw_pwm2, dc2)
		else:
			print("maximum right steer reached")
			pi1.set_PWM_dutycycle(hw_pwm1, dc1)

	elif pressed_keys[K_LEFT]:
		print("key LEFT has been pressed")
		if (dc2 <255):
			dc2 = dc2+1
			pi2.set_PWM_dutycycle(hw_pwm2, dc2)
			if (dc1 >0):
				dc1 = dc1 -1
				pi1.set_PWM_dutycycle(hw_pwm1, dc1)
			else:
				print("maximum left steer reached")
				pi2.set_PWM_dutycycle(hw_pwm2, dc2)
		else:
			print("maximum left steer reached")
			pi2.set_PWM_dutycycle(hw_pwm2, dc2)
	else: #stop the motors if no button is pressed
		#print("position1: {}".format(position1))
		#print("position2: {}".format(position2))
		#time.sleep(0.5)
		speedforward.stop()
		speedbackward.stop()
		pi1.set_PWM_dutycycle(hw_pwm1, dc1) 
		pi2.set_PWM_dutycycle(hw_pwm2, dc2) 
		#pi1.set_servo_pulsewidth(hw_pwm1, curr_pulse1)
		#pi2.set_servo_pulsewidth(hw_pwm2, curr_pulse2)


	pygame.event.pump()

