from keras.models import Sequential
from keras.models import load_model
from matplotlib import pyplot as plt

import numpy as np
import cv2
import scipy.misc as sci
import h5py as h
import os.path
import time
import tensorflow as tf

def nan_mse(output, target):
     sub = tf.sub(output, target)
     sqr = tf.square(sub)
     nans = tf.logical_not(tf.is_nan(target))
     nans_to_zero = tf.cast(nans, tf.float32)
     big_num = tf.constant(10000000.0)
     nans_to_zero_and_big = tf.mul(nans_to_zero, big_num)
     sqr = tf.minimum(sqr, nans_to_zero_and_big)
     num_non_nans = tf.reduce_sum(nans_to_zero) #, axis=-1)
     return tf.div(tf.reduce_sum(sqr), num_non_nans)

model = load_model("model_depth_nan.h5", custom_objects={'nan_mse': nan_mse})

#model.compile(loss=nan_mse, optimizer='adam', metrics = ['accuracy'])

print "Model has been loaded\n"

image_dest = np.load('ny_image.npy')
depth_dest = np.load('ny_depth.npy')

for i in range(image_dest.shape[0]):
    depth_pre = model.predict(np.array(image_dest[i,:,:,:]).reshape((1,320,240,3)), batch_size=1, verbose=1)
    depth_pre = np.resize(depth_pre,(5,5))
    plt.subplot(1,2,1)
    plt.imshow(depth_pre, interpolation='none')
    plt.subplot(1,2,2)
    plt.imshow(image_dest[i,:,:,:])
    print depth_pre
    plt.show()

