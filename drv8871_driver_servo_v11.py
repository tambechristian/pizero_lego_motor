import pygame
from pygame.locals import *
from picamera import PiCamera
from datetime import datetime
import sys

#pygame.init() #move this before pygame command instead, to fix playing sounds

# Assign FPS a value
#FPS = 30
#FramePerSec = pygame.time.Clock()

#pygame.mixer.music.load('0001.mp3')
#pygame.mixer.music.play()
display = pygame.display.set_mode((240, 160)) #240, 160

import time
import RPi.GPIO as GPIO
import pigpio
import os
os.system('sudo pigpiod')
os.system('mpg123 0001.mp3') #neccesary to remove the static noise
#os.system('sudo killall pigpiod') #for debug

time.sleep(1) #wait a few seconds for pigpiod to start

hw_pwm1 = 12
hw_pwm2 = 13
pwm3 = 5
pwm4 = 6
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

led_r = 23
led_b = 24
led_g = 25
led_x = 16
GPIO.setmode(GPIO.BCM)
GPIO.setup(led_r, GPIO.OUT)
GPIO.setup(led_b, GPIO.OUT)
GPIO.setup(led_g, GPIO.OUT)
GPIO.setup(led_x, GPIO.OUT)

GPIO.output(led_r, GPIO.LOW)
GPIO.output(led_b, GPIO.LOW)
GPIO.output(led_g, GPIO.LOW)
GPIO.output(led_x, GPIO.HIGH)

leds_on = False
camera_on = False

#din_pin = 21
#bclk_pin = 18
#lrclk_pin = 19

#GPIO.setup(din_pin, GPIO.OUT)
#GPIO.setup(bclk_pin, GPIO.OUT)
#GPIO.setup(lrclk_pin, GPIO.OUT)

#GPIO.output(din_pin, GPIO.LOW)
#GPIO.output(bclk_pin, GPIO.LOW)
#GPIO.output(lrclk_pin, GPIO.LOW)

#pygame.event.pump()

while 1:
	pressed_keys = pygame.key.get_pressed()

	if pressed_keys[K_q]:
		print("key q (quit) has been pressed")
		pi.hardware_PWM(hw_pwm1, hw_pwm_freq, 0)
		pi.hardware_PWM(hw_pwm2, hw_pwm_freq, 0)
		pi.stop()
		os.system('sudo killall pigpiod')
		pygame.quit()
		sys.exit()
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
	#elif pressed_keys[K_c]:
	#	print("key c (camera) has been pressed ")
	#	if (camera_on == False):
	#		camera = PiCamera()
	#		camera.resolution = (800, 600)
	#		camera.framerate = 24
	#		camera.start_preview()
	#		time.sleep(2)
	#		moment = datetime.now()
	#		camera.start_recording('/home/pi/Videos/Frames/vid_%02d_%02d_%02d.mjpg' % (moment.hour, moment.minute, moment.second))
	#		camera_on = True
	#	else:
	#		camera.stop_preview()
	#		camera.stop_recording()
	#		time.sleep(2)
	#		camera_on = False
	#		camera.close()
	elif pressed_keys[K_l]:
		print("key l (light) has been pressed")
		if (leds_on == False):
			time.sleep(0.5)
			GPIO.output(led_r, GPIO.HIGH)
			GPIO.output(led_b, GPIO.HIGH)
			GPIO.output(led_g, GPIO.HIGH)
			GPIO.output(led_x, GPIO.LOW)
			leds_on = True
		else:
			time.sleep(0.5)
			GPIO.output(led_r, GPIO.LOW)
			GPIO.output(led_b, GPIO.LOW)
			GPIO.output(led_g, GPIO.LOW)
			GPIO.output(led_x, GPIO.HIGH)
			leds_on = False
	elif pressed_keys[K_d]:
		pygame.init()
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
		pygame.init()
		pi.set_PWM_dutycycle(pwm4, 0)
		dc3 = 1500  #1500
		if (dc3 <  boost_dutycycle): #default_dutycycle is the threshhold; it's define at the top
			dc3 = dc3 + 10
			pi.set_PWM_dutycycle(pwm3, dc3)
		else:
			pi.set_PWM_dutycycle(pwm3, dc3)
	elif pressed_keys[K_m]:
		#time.sleep(1)
		os.system('mpg123 /home/pi/Music/tumbler_audio/0001.mp3')
		#os.system('mpg123 aggressive-expansion_edit.mp3')
		time.sleep(0.5)

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

	#FramePerSec.tick(FPS)
	pygame.event.pump()
#pygame.event.pump()

