from picamera import PiCamera
from time import sleep
camera = PiCamera()
camera.resolution = (720, 480)
camera.framerate = 30

camera.start_preview()
sleep(10)
camera.stop_preview()
