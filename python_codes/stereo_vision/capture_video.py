import cv2
print cv2.__version__
import numpy as np
import matplotlib.pyplot as plt

# use : >> v4l2-ctl --list-formats-ext
# to get info about camera attached

# use : >> ls -ltrh /dev/video*
# to get list of usb cameras attached

# https://albertarmea.com/post/opencv-stereo-camera/

left = cv2.VideoCapture(0)
right = cv2.VideoCapture(1)
#width = 1280
#height = 720
#width = 800
#height = 600
#width = 640
#height = 480
width = 320
height = 240
left.set(3, width)
left.set(4, height)
right.set(3, width)
right.set(4, height)

left.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
right.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))

while(True):
    if not (left.grab() and right.grab()):
        print("No more frames")
        break

    _, leftFrame = left.retrieve()
    _, rightFrame = right.retrieve()

    cv2.imshow('left', leftFrame)
    cv2.imshow('right', rightFrame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

left.release()
right.release()
cv2.destroyAllWindows()