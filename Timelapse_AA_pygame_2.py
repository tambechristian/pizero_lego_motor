#import libraries
import pygame
from pygame.locals import *
from picamera import PiCamera
import time

pygame.init()

#Set up variables:
interval = 15
frame = 0

#Set up & start camera, & let it settle
camera = PiCamera()
camera.resolution = (1280, 720)
camera.resolution = (1920, 1080) #fill the whole screen
#camera.resolution = (4056, 3040) # not working
camera.start_preview()
time.sleep(2)

record = 0

while 1:
	pressed_keys = pygame.key.get_pressed()

	if pressed_keys[K_q]:
		pygame.quit()
		exit()

	elif pressed_keys[K_r]:
		if record == 0:
			camera.capture('/home/pi/Videos/Frames/ice_%04d.jpg' % (frame))
			frame = frame + 1
			time.sleep(interval)
	elif pressed_keys[K_t]:
		if record == 1:
			record = 0
			camera.stop_recording()

	pygame.event.pump()


