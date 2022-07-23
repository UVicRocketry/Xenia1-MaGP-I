     
 #This is the draft for the camera stream 

import io
import random
import picamera
import string
import serial 
import time

from time import sleep
from picamera import PiCamera
from smbus import SMBus #to work with 12C communication


pi_camera1 = VideoStream(usePiCamera=True).start()
pi_camera2 = VideoStream(usePiCamera=True).start()

    
if(GPIO_pin==high):
        camera1= picamera.PiCamera()
        camera2= picamera.PiCamera()
        
        stream1 = camera1.PiCameraCircularIO(camera)
        stream2 = camera2.PiCameraCircularIO(camera)
        camera1.start_recording(stream, format='h264')
        try:
                while True:
                        camera.wait_recording(1)
                        camera.wait_recording(10)
                        stream.copy_to('motion.h264')
        finally:
                camera.stop_recording()            
 

