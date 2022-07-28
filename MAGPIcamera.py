 #This is the draft for the camera stream 

import io
import random
import picamera
import string
import serial 
import time
import cv2


import RPi.GPIO as GPIO


from time import sleep
from cv2 import cv2
from picamera import PiCamera
from smbus import SMBus #to work with 12C communication




#start pi camera
def start_picam():
        
        pi_camera = VideoStream(usePiCamera=True).start() #Pi Camera
        
        pi_camera.start_preview()  #to see on laptop
        pi_camera.capture("testPiCam.jpg") #to be commented out
        #camera.vflip=True #in case it is upside down
        time.sleep(5) #before starting recording
        
        pi_camera.start_recording('/home/pi/Desktop/pi_cam_video.h264')   
        
def start_usbcam():
 
 cam = cv2.VideoCapture(0)

 while True:
	 ret, image = cam.read()
	 cv2.imshow('Imagetest',image)
	 k = cv2.waitKey(1)
	 if k != -1:
		break
 cv2.imwrite('/home/pi/testimage.jpg', image)
 cam.release()
 cv2.destroyAllWindows()
 
 
        
     
if (GPIO.input==GPIO.HIGH): #GPIO pin set to trigger camera once received high voltage 
        
        
        try:
                while True:
                        start_picam()
                        start_usbcam()
        finally:
                camera1.stop_recording()       
                camera2.stop_recording()            
                
 
 
 

