from keras.models import Sequential
from keras.models import load_model
from matplotlib import pyplot as plt
from keras import backend as K

import numpy as np
import cv2
import scipy.misc as sci
import h5py as h
import os.path
import time
import tensorflow as tf

def mse_ignore_nan2(y_true, y_pred):
     ok_entries = tf.logical_not(tf.is_nan(y_true))
     safe_targets = tf.where(ok_entries, y_true, y_pred)
     sqr = tf.square(y_pred - safe_targets)

     zero_nans = tf.cast(ok_entries, K.floatx())
     num_ok = tf.reduce_sum(zero_nans, axis=-1) # count OK entries
     num_ok = tf.maximum(num_ok, tf.ones_like(num_ok)) # avoid divide by zero
     return tf.reduce_sum(sqr, axis=-1) / num_ok


#model = load_model("model_depth_nan.h5", custom_objects={'mse_ignore_nan2': mse_ignore_nan2})
model = load_model("model_depth.h5")

#model.compile(loss=nan_mse, optimizer='adam', metrics = ['accuracy'])

print "Model has been loaded\n"

image_dest = np.load('ny_image.npy')
depth_dest = np.load('ny_depth.npy')

for i in range(image_dest.shape[0]):
    depth_pre = model.predict(np.array(image_dest[i,:,:,:]).reshape((1,320,240,3)), batch_size=1)
    depth_pre = np.resize(depth_pre,(5,5))
    """
    sub = plt.subplot(1,2,1)
    depth_plot = plt.imshow(depth_pre, interpolation='none')
    depth_plot.colorbar()
    image_plot = plt.subplot(1,2,2)
    plt.imshow(image_dest[i,:,:,:])
    """

    fig, ax = plt.subplots()

    cax = ax.imshow(depth_pre, interpolation='none')
    fig.colorbar(cax)

    plt.subplots()
    plt.imshow(image_dest[i,:,:,:])

    print depth_pre
    plt.show()

