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
hw_freq = 200000  #400000  -> 8000

pi1 = pigpio.pi()
pi2 = pigpio.pi()

pi1.set_mode(hw_pwm1, pigpio.OUTPUT)
pi2.set_mode(hw_pwm2, pigpio.OUTPUT)

pi1.set_PWM_range(hw_pwm1, 255)
pi2.set_PWM_range(hw_pwm2, 255)

pi1.set_PWM_frequency(hw_pwm1, 400000)
print("hw pwm1 freq: {}".format(pi1.get_PWM_frequency(hw_pwm1)))
pi2.set_PWM_frequency(hw_pwm2, 400000)
print("hw pwm2 freq: {}".format(pi2.get_PWM_frequency(hw_pwm2)))

dc1 = 128
dc2 = 128
current_dc1 = 128
current_dc2 = 128
#servo1 = Servo(hw_pwm1, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000, pin_factory=factory)
#servo1 = Servo(hw_pwm1, pin_factory = factory)
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

steps = 1500
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
	pressed_keys = pygame.key.get_pressed()

	if pressed_keys[K_q]:
		print("key q (quit) has been pressed")
		pi1.set_PWM_dutycycle(hw_pwm1, 128)
		pi2.set_PWM_dutycycle(hw_pwm2, 128)
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
		#pi2.set_servo_pulsewidth(hw_pwm2, 0)
		pi2.set_PWM_dutycycle(hw_pwm2, 0)
		for dc1 in range (dc1, 255, 18): #18 = (255-128) /7
			pi1.set_PWM_dutycycle(hw_pwm1, dc1)
			current_dc1 = dc1
		#for steps in range (steps, 2500, 286): #2500 - 500 = 2000 / 7(servo lego steps)  =285.7
		#	pi1.set_servo_pulsewidth(hw_pwm1, steps)
	elif pressed_keys[K_LEFT]:
		print("key LEFT has been pressed")
		pi1.set_PWM_dutycycle(hw_pwm1, 0)
		for dc2 in range(dc2, 0, -18):
			pi2.set_PWM_dutycycle(hw_pwm2, dc2)
			current_dc2 = dc2
		#pi1.set_servo_pulsewidth(hw_pwm1, 0)
		#for steps in range (steps, 500, -286):
			#pi2.set_servo_pulsewidth(hw_pwm2, steps)
	else: #stop the motors if no button is pressed
		print("current dc1: {}".format(current_dc1))
		print("current dc2: {}".format(current_dc2))
		time.sleep(0.5)
		speedforward.stop()
		speedbackward.stop()
		pi1.set_PWM_dutycycle(hw_pwm1, current_dc1) 
		pi2.set_PWM_dutycycle(hw_pwm2, current_dc2) 
		#pi1.set_servo_pulsewidth(hw_pwm1, 0)
		#pi2.set_servo_pulsewidth(hw_pwm2, 0)


	pygame.event.pump()

