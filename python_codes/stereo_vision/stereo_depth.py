import sys
import numpy as np
import cv2
from matplotlib import pyplot as plt

REMAP_INTERPOLATION = cv2.INTER_LINEAR
DEPTH_VISUALIZATION_SCALE = 1

if len(sys.argv) != 2:
    print("Syntax: {0} CALIBRATION_FILE".format(sys.argv[0]))
    sys.exit(1)

calibration = np.load(sys.argv[1], allow_pickle=False)
imageSize = tuple(calibration["imageSize"])
leftMapX = calibration["leftMapX"]
leftMapY = calibration["leftMapY"]
leftROI = tuple(calibration["leftROI"])
rightMapX = calibration["rightMapX"]
rightMapY = calibration["rightMapY"]
rightROI = tuple(calibration["rightROI"])

CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 720

cv2.namedWindow('right')
cv2.namedWindow('left')
cv2.namedWindow('depth')
cv2.moveWindow('right', 0,0)
cv2.moveWindow('left', 960,300)
cv2.moveWindow('depth', 500,300)

right = cv2.VideoCapture(1)
left = cv2.VideoCapture(0)

# Increase the resolution
right.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
right.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
left.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
left.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)

# Use MJPEG to avoid overloading the USB 2.0 bus at this resolution
right.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
left.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))

# The distortion in the left and right edges prevents a good calibration, so
# discard the edges
CROP_WIDTH = 960
def cropHorizontal(image):
    return image[:, int((CAMERA_WIDTH-CROP_WIDTH)/2): int(CROP_WIDTH+(CAMERA_WIDTH-CROP_WIDTH)/2)]

# TODO: Why these values in particular?
# TODO: Try applying brightness/contrast/gamma adjustments to the images
stereoMatcher = cv2.StereoBM_create()
#stereoMatcher.setMinDisparity(4)
#stereoMatcher.setNumDisparities(128)
#stereoMatcher.setBlockSize(21)
#stereoMatcher.setROI1(leftROI)
#stereoMatcher.setROI2(rightROI)
#stereoMatcher.setSpeckleRange(16)
#stereoMatcher.setSpeckleWindowSize(45)
stereoMatcher.setMinDisparity(16)
stereoMatcher.setNumDisparities(256)
stereoMatcher.setBlockSize(21)
stereoMatcher.setROI1(leftROI)
stereoMatcher.setROI2(rightROI)
stereoMatcher.setSpeckleRange(16)
stereoMatcher.setSpeckleWindowSize(45)

# Grab both frames first, then retrieve to minimize latency between cameras
while(True):
    if not left.grab() or not right.grab():
        print("No more frames")
        break

    _, rightFrame = right.retrieve()
    rightFrame = cropHorizontal(rightFrame)
    rightHeight, rightWidth = rightFrame.shape[:2]
    _, leftFrame = left.retrieve()
    leftFrame = cropHorizontal(leftFrame)
    leftHeight, leftWidth = leftFrame.shape[:2]

    if (rightWidth, rightHeight) != imageSize:
        print("Right camera has different size than the calibration data")
        break
    if (leftWidth, leftHeight) != imageSize:
        print("Left camera has different size than the calibration data")
        break

    fixedRight = cv2.remap(rightFrame, rightMapX, rightMapY, REMAP_INTERPOLATION)
    fixedLeft = cv2.remap(leftFrame, leftMapX, leftMapY, REMAP_INTERPOLATION)

    grayRight = cv2.cvtColor(fixedRight, cv2.COLOR_BGR2GRAY)
    grayLeft = cv2.cvtColor(fixedLeft, cv2.COLOR_BGR2GRAY)
    depth = stereoMatcher.compute(grayRight, grayLeft)

    cv2.imshow('right', fixedRight)
    cv2.imshow('left', fixedLeft)
    cv2.imshow('depth', depth / DEPTH_VISUALIZATION_SCALE)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

right.release()
left.release()
cv2.destroyAllWindows()