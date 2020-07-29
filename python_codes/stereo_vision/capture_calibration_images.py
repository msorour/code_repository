import cv2
import time

LEFT_PATH = "capture/left/{:06d}.jpg"
RIGHT_PATH = "capture/right/{:06d}.jpg"

CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 720

left = cv2.VideoCapture(0)
right = cv2.VideoCapture(1)

# Increase the resolution
right.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
right.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
left.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
left.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)

# Use MJPEG to avoid overloading the USB 2.0 bus at this resolution
right.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
left.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))

# The distortion in the left and right edges prevents a good calibration, so discard the edges:
CROP_WIDTH = 960
def cropHorizontal(image):
    return image[:, int((CAMERA_WIDTH-CROP_WIDTH)/2): int(CROP_WIDTH+(CAMERA_WIDTH-CROP_WIDTH)/2)]

frameId = 0
frameIdtoSave = 0
number_of_calibration_frames = 100
number_of_images_to_skip = 50
total_number_of_images = number_of_calibration_frames*number_of_images_to_skip

cv2.namedWindow('right')
cv2.namedWindow('left')
cv2.moveWindow('right', 0,0)
cv2.moveWindow('left', 1000,500)

# Grab both frames first, then retrieve to minimize latency between cameras
while(True and frameId<total_number_of_images):
    if not (right.grab() and left.grab()):
        print("No more frames")
        break

    if(frameId<10):
        time.sleep(1)

    _, rightFrame = right.retrieve()
    _, leftFrame = left.retrieve()

    rightFrame = cropHorizontal(rightFrame)
    leftFrame = cropHorizontal(leftFrame)

    cv2.imshow('right', rightFrame)
    cv2.imshow('left', leftFrame)

    if((frameId%number_of_images_to_skip)==0):
        cv2.imwrite(RIGHT_PATH.format(frameIdtoSave), rightFrame)
        cv2.imwrite(LEFT_PATH.format(frameIdtoSave), leftFrame)
        frameIdtoSave += 1



    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    frameId += 1

right.release()
left.release()
cv2.destroyAllWindows()