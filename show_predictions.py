from keras.models import Sequential
from keras.models import load_model
from matplotlib import pyplot as plt

import numpy as np
import cv2
import scipy.misc as sci
import h5py as h
import os.path
import time

model = load_model("depth_net.h5")


print "Model has been loaded\n"

image_dest = np.load('ny_image.npy')
depth_dest = np.load('ny_depth.npy')

for i in range(image_dest.shape[0]):
    depth_pre = model.predict(np.array(image_dest[i,:,:,:]).reshape((1,320,240,3)), batch_size=1, verbose=1)
    depth_pre = np.resize(depth_pre,(80,60))
    plt.subplot(1,2,1)
    plt.imshow(depth_pre)
    plt.subplot(1,2,2)
    plt.imshow(image_dest[i,:,:,:])

    plt.show()
    
    
