#import libraries
import pygame
from pygame.locals import *
from picamera import PiCamera
from datetime import datetime
import time

pygame.init()
display = pygame.display.set_mode((300, 300))

#Set up variables:
interval = 15
frame = 0

#Set up & start camera, & let it settle
#camera = PiCamera()
#camera.resolution = (640, 360)
#camera.resolution = (800, 600)
#camera.resolution = (1280, 720)
#camera.resolution = (1920, 1080) #fill the whole screen
#camera.resolution = (4056, 3040) # not working
#camera.framerate = (25)

#camera.start_preview() #works but no control
#time.sleep(2)

record = 0

while 1:
	pressed_keys = pygame.key.get_pressed()

	if pressed_keys[K_q]:
		print("key q (quit) has been pressed")
		pygame.quit()
		exit()

	elif pressed_keys[K_r]:
		print("key r (record) has been pressed")
		if record == 0:
			record = 1
			camera = PiCamera()
			camera.resolution = (800, 600)
			camera.framerate = 24
			camera.start_preview()
			time.sleep(2)
			moment = datetime.now()
			#camera.capture('/home/pi/Videos/Frames/ice_%04d.jpg' % (frame))
			camera.start_recording('/home/pi/Videos/Frames/vid_%02d_%02d_%02d.mjpg' % (moment.hour, moment.minute, moment.second))
			#frame = frame + 1 
			#time.sleep(interval)
	elif pressed_keys[K_t]:
		print('key t has been pressed')
		if record == 1:
			record = 0
			camera.stop_preview()
			camera.stop_recording()
			time.sleep(2)
			camera.close()

	pygame.event.pump()


