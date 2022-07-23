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


camera = PiCamera()
camera.resolution = (1024, 760)
camera.start_preview()
camera.start_recording('video.h264')
sleep(5)
camera.stop_recording()
camera.stop_preview()

def start_serial():
    ser=serial.Serial('/dev/ttyACM!', 9600,timeout=1)
    #ttyACM1 is the name of arduino in port,might need to change
    #timeout to protect program
    while True:
        ser_data=ser.readline() 
        ser_data.flush()
        return ser_data
    #store ser_data for function use 
    

def motion_detected():
    data= start_serial()
    if(data!="Acceleration data"):
        accel_x, accel_y, accel_z=data 
        if (int(accel_x)!=0 and int(accel_y)!=0 and int(accel_z)!=0):
            camera = picamera.PiCamera()
            stream = picamera.PiCameraCircularIO(camera, seconds=20)
            camera.start_recording(stream, format='h264')
            try:
                while True:
                    camera.wait_recording(1)
                    if motion_detected():
                        # Keep recording for 10 seconds and only then write the
                        # stream to disk
                        camera.wait_recording(10)
                        stream.copy_to('motion.h264')
            finally:
                camera.stop_recording()            
 
