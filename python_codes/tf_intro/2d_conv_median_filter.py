import cv2
print cv2.__version__
import numpy as np
import matplotlib.pyplot as plt
import scipy.signal

img = cv2.imread('monalisa.jpg')
gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

row, col = 1192, 800

# generating salt and pepper noise
np.random.seed(0)
gray_sp = gray*1 # sp stands for salt and pepper
sp_indices = np.random.randint(0,21,[row,col])
for i in xrange(row):
    for j in xrange(col):
        if sp_indices[i,j] == 0:
            gray_sp[i,j] = 0
        if sp_indices[i,j] == 20:
            gray_sp[i,j] = 255

# Now we want to remove salt and pepper noise using median filter
gray_sp_removed = cv2.medianBlur(gray_sp,3)

f, (ax1, ax2, ax3) = plt.subplots(1, 3, sharey=True)
ax1.imshow(gray,cmap='gray')
ax1.set_title('original image')
ax2.imshow(gray_sp,cmap='gray')
ax2.set_title('salt and pepper noisy')
ax3.imshow(gray_sp_removed,cmap='gray')
ax3.set_title('median filter 3X3')
plt.show()


cam = cv2.VideoCapture(0)
#cam1 = cv2.VideoCapture(1)

cv2.namedWindow("test")

img_counter = 0
#img_counter1 = 0

while True:
    ret, frame = cam.read()
    #ret1, frame1 = cam1.read()
    cv2.imshow("test", frame)
    #cv2.imshow("test1", frame1)

    if not ret:
        break
    k = cv2.waitKey(1)

    if k%256 == 27:
        # ESC pressed
        print("Escape hit, closing...")
        break
    elif k%256 == 32:
        # SPACE pressed
        img_name = "opencv_frame_{}.png".format(img_counter)
        #img_name1 = "opencv_frame_{}.png".format(img_counter1)

        cv2.imwrite(img_name, frame)
        #cv2.imwrite(img_name1, frame1)
        print("{} written!".format(img_name))
        #print("{} written!".format(img_name1))
        img_counter += 1
        #img_counter1 += 1

cam.release()
#cam1.release()

cv2.destroyAllWindows()