from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np


camera = PiCamera()
camera.resolution = (640, 368)
camera.rotation = 180
rawCapture = PiRGBArray(camera, size=(640, 360))
time.sleep(0.1)

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):	
	image = frame.array
	roi = image[200:300, 0:639]
	Blackline= cv2.inRange(roi, (0,0,0), (50,50,50))
	img,contours = cv2.findContours(Blackline.copy(),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	cv2.drawContours(image,contours,-1,(0,255,0),3)
	cv2.imshow("orginal with line", image)	
	rawCapture.truncate(0)	
	key = cv2.waitKey(1) & 0xFF	
	if key == ord("q"):
		break

GPIO.output(40, GPIO.LOW)