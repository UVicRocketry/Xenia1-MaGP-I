#This is the draft for the camera stream 

import io
import random
import picamera
import string
import time
import cv2

import RPi.GPIO as GPIO

from time import sleep

from picamera import PiCamera

#start pi camera
def start_picam():
    pi_camera = picamera.PiCamera()
    pi_camera.resolution = (1024,720)
    pi_camera.start_preview()
    picam_filename = str(time.time()) + ".h264"
    pi_camera.start_recording(picam_filename)
    while GPIO.input == GPIO.HIGH:
        pi_camera.wait_recording(5) #if still deployed record 5 more seconds
    pass
    pi_camera.stop_recording()
    pi_camera.stop_preview()



def start_usbcam():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("cannot open cam")
        exit()
    while True:
        ret, frame = cap.read()

        if not ret:
            print("cannot recieve frame")
            break
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        cv2.imshow('frame', gray)
        if cv2.waitKey(1) == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()

mode = GPIO.getmode()
GPIO.setmode(mode)
GPIO.setup(8, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) #NC low (0V)

while GPIO.input == GPIO.LOW:
    pass

start_picam()
#start_usbcam()

#timer

#end record and store video
