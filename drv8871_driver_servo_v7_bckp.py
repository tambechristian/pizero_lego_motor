import pygame
from pygame.locals import *
import sys

pygame.init()
display = pygame.display.set_mode((300, 300))

import time
import RPi.GPIO as GPIO
import pigpio
import os
os.system('sudo pigpiod')
#os.system('sudo killall pigpiod') #for debug

time.sleep(1) #wait a few seconds for pigpiod to start

hw_pwm1 = 12
hw_pwm2 = 13
pwm3 = 26
pwm4 = 19
hw_pwm_freq = 2500

pi = pigpio.pi() #pigpio start to initilize

time.sleep(1) #wait a few second to finish  initializing  the 4 Pi connections

pi.set_mode(hw_pwm1, pigpio.OUTPUT)
pi.set_mode(hw_pwm2, pigpio.OUTPUT)
pi.set_mode(pwm3, pigpio.OUTPUT)
pi.set_mode(pwm4, pigpio.OUTPUT)

pi.set_PWM_range(pwm3, 3000) # setting the pwm range of L motors
pi.set_PWM_range(pwm4, 3000) # setting the pwm range of L motors
pi.set_PWM_frequency(pwm3, 1200)
print("gpio pwm3 freq: {}".format(pi.get_PWM_frequency(pwm3)))
pi.set_PWM_frequency(pwm4, 1200)
print("gpio pwm4 freq: {}".format(pi.get_PWM_frequency(pwm4)))

dc1 = 0  #dutycycle for servo left
dc2 = 0  #dutyccle for servo right
dc3 = 0  #dutycycle for motors forward direction
dc4 = 0  #dutycycle for motors reverse direction

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

#dc_default = 9.00 * lmotors_max_volt_DutyCycle / lmotors_max_volt
default_dutycycle = (3000 * (lmotors_max_volt - 3)) / batt_volt #3000 is the max range set. -3 to get o 9V
print("default duty cycle is: {}".format(default_dutycycle))
boost_dutycycle = (3000 * lmotors_max_volt) / batt_volt #3000 is the max range set
print("boost duty cycle is: {}".format(boost_dutycycle))
#default_dutycycle = 1500  #1500 dc outputs 9V with battery at 15.6V
#boost_dutycycle = 1688  #2250 is 3/4 on from 3000 range. it outputs about 12V with battery at 15.69V

pwm_freq = 1200
vout_default = 9
vout_max = 12

led_ain1 = 21
led_ain2 = 20
led_en = 16
GPIO.setmode(GPIO.BCM)
GPIO.setup(led_ain1, GPIO.OUT)
GPIO.setup(led_ain2, GPIO.OUT)
GPIO.setup(led_en, GPIO.OUT)

GPIO.output(led_en, GPIO.HIGH)

leds_on = False

while 1:
	pressed_keys = pygame.key.get_pressed()

	if pressed_keys[K_q]:
		print("key q (quit) has been pressed")
		pi.hardware_PWM(hw_pwm1, hw_pwm_freq, 0)
		pi.hardware_PWM(hw_pwm2, hw_pwm_freq, 0)
		pi.stop()
		os.system('sudo killall pigpiod')
		pygame.quit()
		exit()
	elif pressed_keys[K_i]:
		time.sleep(0.5)
		print("key i (info) has been pressed")
		batt_volt = ((chan.voltage) * 37500 / 7500)
		lmotors_max_volt = batt_volt / volt_scale_factor
		default_dutycycle = (3000 * (lmotors_max_volt - 3)) / batt_volt 
		print("battery voltage: {}".format(batt_volt))
		print("lmotors_max_volt: {}".format(lmotors_max_volt))
		print("default dutycycle: {}".format(default_dutycycle))
		print("Enter a command \n")
		if (batt_volt) <9.0:
			print("low battery!!!")
	elif pressed_keys[K_l]:
		print("key l (light) has been pressed")
		if (leds_on == False):
			time.sleep(0.5)
			GPIO.output(led_ain1, GPIO.HIGH)
			GPIO.output(led_ain2, GPIO.HIGH)
			leds_on = True
		else:
			time.sleep(0.5)
			GPIO.output(led_ain1, GPIO.LOW)
			GPIO.output(led_ain2, GPIO.LOW)
			leds_on = False
	elif pressed_keys[K_o]:
		print("keys o (light off) has been pressed")
		time.sleep(0.05)
		GPIO.output(led_ain1, GPIO.LOW)
		GPIO.output(led_ain2, GPIO.LOW)
	elif pressed_keys[K_d]:
		#print("key d (drive) has been pressed")
		#pi3.set_PWM_dutycycle(pwm3, dc_default)
		#pi4.set_PWM_dutycycle(pwm4, 0)
		pi.set_PWM_dutycycle(pwm4, 0)
		dc3 = 1000  #1500
		dc3 = dc3 + 10
		if (dc3 < default_dutycycle): #default_dutycycle is the threshhold; it's define at the top
			#dc3 = dc3 + 10
			pi.set_PWM_dutycycle(pwm3, dc3)
		else:
			dc3 = dc3
			#pi.set_PWM_dutycycle(pwm3, dc3)

		if pressed_keys[K_d] and pressed_keys[K_LEFT]:
			#print("both keys d and key left have been pressed")
			dc2 = dc2 + 495
			if (dc2 <1000000):
				#dc2 = dc2 + 500
				pi.hardware_PWM(hw_pwm2, hw_pwm_freq, dc2)
			#else:
			#	pi.hardware_PWM(hw_pwm2, hw_pwm_freq, dc2)
		else:
			dc2 = 0
			pi.hardware_PWM(hw_pwm2, hw_pwm_freq, dc2) #resetting servo motors once right key is released

		if pressed_keys[K_d] and pressed_keys[K_RIGHT]:
			#print("both key d and key right are pressed")
			dc1 = dc1 + 495
			if (dc1 < 1000000):
				#dc1 = dc1 + 500
				pi.hardware_PWM(hw_pwm1, hw_pwm_freq, dc1)
			#else:
			#	pi.hardware_PWM(hw_pwm1, hw_pwm_freq, dc1)
		else:
			dc1 = 0
			pi.hardware_PWM(hw_pwm1, 1000, dc1) #resetting servo motors once left key is released

	elif pressed_keys[K_r]:
		#print("key r (reverse) has been pressed")
		pi.set_PWM_dutycycle(pwm3, 0)
		dc4 = 1000
		dc4 = dc4 +10
		if (dc4 < default_dutycycle):
			#dc4 = dc4 + 10
			pi.set_PWM_dutycycle(pwm4, dc4)
		else:
			dc4 = dc4
			#pi.set_PWM_dutycycle(pwm4, dc4)

	elif pressed_keys[K_RIGHT]:
		dc1 = dc1 + 495
		if (dc1 < 1000000):
			#dc1 = dc1 + 495
			pi.hardware_PWM(hw_pwm1, hw_pwm_freq, dc1)
		else:
			dc1 = dc1
		#	pi.hardware_PWM(hw_pwm1, hw_pwm_freq, dc1-495)

	elif pressed_keys[K_LEFT]:
		dc2 = dc2 + 495
		if (dc2 < 1000000):
			#dc2 = dc2 + 495
			pi.hardware_PWM(hw_pwm2, hw_pwm_freq, dc2)
		else:
			dc2 = dc2

	elif pressed_keys[K_b]:
		pi.set_PWM_dutycycle(pwm4, 0)
		dc3 = 1500  #1500
		if (dc3 <  boost_dutycycle): #default_dutycycle is the threshhold; it's define at the top
			dc3 = dc3 + 10
			pi.set_PWM_dutycycle(pwm3, dc3)
		else:
			pi.set_PWM_dutycycle(pwm3, dc3)

	else:
		# resetting servo motors
		dc1 = 0
		dc2 = 0
		pi.hardware_PWM(hw_pwm1, hw_pwm_freq, dc1)
		pi.hardware_PWM(hw_pwm2, hw_pwm_freq, dc2)

		pi.set_PWM_dutycycle(pwm3, dc3)
		pi.set_PWM_dutycycle(pwm4, dc4)
		#gradually resetting l motors
		if (dc3 > 0):
			dc3 = dc3 - 2
			pi.set_PWM_dutycycle(pwm3, dc3)
		#else:
		#	pi.set_PWM_dutycycle(pwm3, dc3) 
		if (dc4 > 0):
			dc4 = dc4 - 2
			pi.set_PWM_dutycycle(pwm4, dc4) 
		#else:
		#	pi.set_PWM_dutycycle(pwm4, dc4)  

	pygame.event.pump()

