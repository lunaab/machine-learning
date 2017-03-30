import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from PIL import Image
import time
import sys
import cv2

if len(sys.argv) != 2:
    print "Usage: \"python view_data.py <file.npz>\""
    exit(-1)

file = sys.argv[1]

data = np.load(file)

images = data['arr_0']
depths = data['arr_1']
motion = data['arr_2']

print "Shapes (im, depth, motion): ", images.shape, depths.shape, motion.shape

for i in range(images.shape[0]):
    plt.subplot(1,2,1)
    plt.imshow(depths[i,:,:])
    plt.subplot(1,2,2)
    plt.imshow(images[i,:,:,::-1]/255.0)
#    plt.imshow(cv2.cvtColor(images[i,:,:,], cv2.COLOR_BGR2RGB))
    print motion[i,:,:]
    plt.show()


"""
for i in images:
  im = Image.fromarray(i, 'RGB')
  im.show()
  time.sleep(2)

print images.shape
print images[0]
"""
