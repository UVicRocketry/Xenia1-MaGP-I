 #This is the draft for the camera stream 

import io
import random
import picamera
import string
import serial 
import time
#import cv2


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
        usb_camera = VideoStream(src=0).start() #USB camera 
        
        usb_camera.start_preview()  #to see on laptop
        usb_camera.capture("testUSBCam.jpg")         
        
        usb_camera.start_recording('/home/pi/Desktop/usb_cam_video.h264')
        
        
       
    
if (GPIO.input==GPIO.HIGH): #GPIO pin set to trigger camera once received high voltage 
        
        
        try:
                while True:
                        start_picam()
                        start_usbcam()
        finally:
                camera1.stop_recording()       
                camera2.stop_recording()            
                
 
 
 

