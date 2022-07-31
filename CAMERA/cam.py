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
    pi_camera.resolution = (640,480)
    pi_camera.start_preview()
    pi_camera.start_recording('recorded.h264')
    pi_camera.wait_recording(2)
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


start_picam()
start_usbcam()
"""
if (GPIO.input==GPIO.HIGH): #GPIO pin set to trigger camera once received high voltage 

    try:
        while True:
            start_picam()
            start_usbcam()
    finally:
            camera1.stop_recording()
            camera2.stop_recording()
"""
