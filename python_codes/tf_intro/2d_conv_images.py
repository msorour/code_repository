import cv2
print cv2.__version__
import numpy as np
import matplotlib.pyplot as plt
import scipy.signal

img = cv2.imread('monalisa.jpg')
gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

mean = 0
var = 100
sigma = var*0.5
row, col = 1192, 800

gauss = np.random.normal(mean,sigma,(row,col))
gauss = gauss.reshape(row,col)
gray_noisy = gray+gauss

## Mean Filter
Hm = np.array([[1,1,1],[1,1,1],[1,1,1]])/float(9)
Gm = scipy.signal.convolve2d(gray_noisy,Hm,mode='same')

f, (ax1, ax2, ax3) = plt.subplots(1, 3, sharey=True)
ax1.imshow(gray,cmap='gray')
ax1.set_title('original gray')
ax2.imshow(gray_noisy,cmap='gray')
ax2.set_title('noisy')
ax3.imshow(Gm,cmap='gray')
ax3.set_title('mean')
plt.show()