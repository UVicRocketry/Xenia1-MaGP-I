#code for raspberry cameras

from picamera import PiCamera
import time

cam = PiCamera()
time.sleep(3) #to adjust lens and brightness
camera.resolution = (1280, 720) #camera definition in width and height
camera.rotation = 180 #to ensure the image isn't flipped, comment out if needed
camera.contrast = -10 #so that sky brightness is balanced
video_save = "/home/pi/Pictures/video_" + str(time.time()) + ".h264" #save and timestamp

print("Start recording...")
camera.start_recording(video_save)
camera.wait_recording(1400) #parameter- time in seconds for recording
camera.stop_recording()
print("Done.")
