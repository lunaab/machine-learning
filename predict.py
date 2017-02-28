from keras.models import Sequential
from keras.models import load_model

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
new_image = sci.imread('images/rand_room.jpeg')

print np.array(new_image).shape

start = time.clock()
depth_pre_new = model.predict(np.array([sci.imresize(np.array(new_image), (320,240,3))]), batch_size=1, verbose=1)
stop = time.clock() - start

start = time.clock()
depth_pre = model.predict(np.array([sci.imresize(image_dest[-1,...], (320,240,3))]), batch_size=1, verbose=1)
stop = time.clock() - start

print "*******Time Elapsed*********\n", stop


print depth_pre
#print "\n*****Normalized For visualization*****\n", depth_dest[-1]
#print "\n*****Actual*****\n", depth_dest[-1]
print "\n*****MaxError*****\n:", max(max(abs(depth_pre-depth_dest[-1])))

depth_pre = ((depth_pre/max(max(depth_pre)))*255)
depth_pre = np.resize(depth_pre,(5,5))
#depth_act = depth_dest[-1,:]
#depth_act = ((depth_act/max(depth_act))*255)
#depth_act = np.resize(depth_act,(80,60))
print "\n*****Reshaped********\n", depth_pre

sci.imsave('images/predict_depth_rand.jpg', depth_pre_new)
sci.imsave('images/predict_depth.jpg', depth_pre)
#sci.imsave('last_image.jpg', image_dest[-1,:,:,:])

"""
def depth_to_pixel(depths):
    return (depths/max(max(depth_pre)))*255
"""
