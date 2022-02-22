import pygame
from pygame.locals import *
import sys

pygame.init()
display = pygame.display.set_mode((300, 300))

from picamera import PiCamera
from datetime import datetime

#setup camera
camera = PiCamera()
camera.resolution = (1280, 720)
camera.framerate = (25)

record = 0

while True:
	pressed_keys = pygame.key.get_pressed()
	if pressed_keys [K_q]:
		print("key q (quit) has been pressed")
		#pi.stop()
		pygame.quit()
		exit()
	elif pressed_keys[K_c]:
		print("key c (camera) has been pressed")
		if  record == 0:
			record = 1
			moment = datetime.now()
			camera.start_recording('/home/pi/Videos/vid_%02d_%02d_%02d.mjpg' % (moment.hour, moment.minute, moment.second))
	elif pressed_keys[K_t]:
		print ("key t (terminate camera) has been pressed")
		if record == 1:
			record = 0
			camera.stop_recording()
	pygame.event.pump()
