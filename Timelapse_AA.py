#import libraries
from picamera import PiCamera
import time

#Set up variables:
interval = 15
frame = 0

#Set up & start camera, & let it settle
camera = PiCamera()
camera.resolution = (1920, 1080)
#camera.resolution = (4056, 3040)
camera.start_preview()
time.sleep(2)

while True:
	camera.capture('/home/pi/Videos/Frames/ice_%04d.jpg' % (frame))
	frame = frame + 1
	time.sleep(interval)


