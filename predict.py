from keras.models import Sequential
from keras.models import load_model

import numpy as np
import cv2
import scipy.misc as sci
import h5py as h
import os.path

model = load_model("depth_net.h5")


print "Model has been loaded\n"

image_dest = np.load('ny_image.npy')
depth_dest = np.load('ny_depth.npy')

depth_pre = model.predict(np.array(image_dest[-1,:,:,:]).reshape((1,320,240,3)), batch_size=1, verbose=1)

print depth_pre
print "\n*****Normalized For visualization*****\n", ((depth_pre/max(max(depth_pre)))*255).shape
print "\n*****Actual*****\n", depth_dest[-1]
print "\n*****MaxError*****\n:", max(max(abs(depth_pre-depth_dest[-1])))

depth_pre = ((depth_pre/max(max(depth_pre)))*255)
depth_pre = np.resize(depth_pre,(80,60))

print "\n*****Reshaped********\n", depth_pre

sci.imsave('predict_depth.jpg', depth_pre)

"""
def depth_to_pixel(depths):
    return (depths/max(max(depth_pre)))*255
"""
