import numpy as np
import cv2
import scipy.misc as sci
import h5py as h
import os.path
import time
import sys

if len(sys.argv) != 2:
  sys.exit("Usage: python predict.py <\"model-file-path\">")

from keras.models import load_model
from keras.models import Sequential

#loads the desired model to predict
model = load_model(sys.argv[1])

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
depth_pre_new = ((depth_pre_new/max(max(depth_pre_new)))*255)
depth_pre_new = np.resize(depth_pre_new,(5,5))
#depth_act = depth_dest[-1,:]
#depth_act = ((depth_act/max(depth_act))*255)
#depth_act = np.resize(depth_act,(80,60))
print "\n*****Reshaped********\n", depth_pre
print depth_pre_new.shape
print depth_pre.shape


sci.imsave('images/predict_depth_rand.jpg', depth_pre_new)
sci.imsave('images/predict_depth.jpg', depth_pre)
#sci.imsave('last_image.jpg', image_dest[-1,:,:,:])

"""
def depth_to_pixel(depths):
    return (depths/max(max(depth_pre)))*255
"""
