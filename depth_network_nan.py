from keras.models import Sequential
from keras.layers import Dense
from keras.layers import Convolution2D
from keras.layers.pooling import MaxPooling2D
from keras.layers.core import Flatten
from keras.layers.core import Dropout
from keras.optimizers import SGD
from keras import backend as K

import tensorflow as tf
import numpy as np
import cv2
import scipy.misc as sci
import h5py as h
import os.path

#Set to true to save model after training
save_model = True

image_dest = np.load('image_data.npy')
depth_dest = np.load('depth_data.npy')

def mse_ignore_nan(y_true, y_pred):
    sqr = tf.square(y_pred - y_true)
    ok_entries = tf.logical_not(tf.equal(y_true,0))
    zero_nans = tf.cast(ok_entries, K.floatx())
    max_float = tf.constant(np.finfo(K.floatx()).max)
    sqr_ok = tf.minimum(sqr, zero_nans * max_float)
    num_ok = tf.reduce_sum(zero_nans, axis=-1)
    num_ok = tf.maximum(num_ok, tf.ones_like(num_ok))
    return tf.reduce_sum(sqr_ok, axis=-1) / num_ok #also had an axis

def nan_mse(output, target):
     sub = tf.sub(output, target)
     sqr = tf.square(sub)
     nans = tf.logical_not(tf.is_nan(target))
     nans_to_zero = tf.cast(nans, tf.float32)
     big_num = tf.constant(1000000.0)
     nans_to_zero_and_big = tf.mul(nans_to_zero, big_num)
     sqr = tf.minimum(sqr, nans_to_zero_and_big)
     num_non_nans = tf.reduce_sum(nans_to_zero) #, axis=-1)
     return tf.div(tf.reduce_sum(sqr), num_non_nans)

def mse_ignore_nan1(y_true, y_pred):
     sqr = tf.square(y_pred - y_true)
     sqr_no_nans = tf.where(tf.equal(y_true, 0.0), tf.zeros_like(sqr), sqr)
     s = tf.reduce_sum(sqr_no_nans, axis=-1)

def mse_ignore_nan2(y_true, y_pred):
     ok_entries = tf.logical_not(tf.is_nan(y_true))
     safe_targets = tf.where(ok_entries, y_true, y_pred)
     sqr = tf.square(y_pred - safe_targets)

     zero_nans = tf.cast(ok_entries, K.floatx())
     num_ok = tf.reduce_sum(zero_nans, axis=-1) # count OK entries
     num_ok = tf.maximum(num_ok, tf.ones_like(num_ok)) # avoid divide by zero
     return tf.reduce_sum(sqr, axis=-1) / num_ok

#here the model architecture is defined
#using TS for background so depth data has to be ordered (width, heighth, depth)
model = Sequential()

sgd = SGD(lr=0.01, decay=1e-6, momentum=0.9)

# Coarse 1
model.add(Convolution2D(96,11,11,activation='relu',input_shape=(320, 240,3),dim_ordering='tf',
                        subsample=(4,4)))
model.add(MaxPooling2D(pool_size=(2,2)))

#Coarse 2
model.add(Convolution2D(256,5,5,activation='relu',dim_ordering='tf'))#,input_shape=(40,30,3)
model.add(MaxPooling2D(pool_size=(2,2)))

#Coarse 3-4
model.add(Convolution2D(384,3,3,activation='relu',dim_ordering='tf'))#,input_shape=(20,15,3)
model.add(Convolution2D(384,3,3,activation='relu',dim_ordering='tf'))#,input_shape=(20,15,3)

#Coarse 5
model.add(Convolution2D(256,3,3,activation='relu',dim_ordering='tf'))#,input_shape=(20,15,3)
model.add(MaxPooling2D(pool_size=(2,2)))

#Coarse 6
model.add(Flatten())
model.add(Dropout(.1))
model.add(Dense(4096, init='uniform', activation='relu'))
#model.add(Dense(80*60, init='uniform', activation='relu'))

model.add(Dense(5*5, init='uniform', activation='linear'))

#compile the model
model.compile(loss=mse_ignore_nan2, optimizer=sgd, metrics = ['accuracy'])

#fit the model to matching depth data
model.fit(image_dest, depth_dest, nb_epoch=60, batch_size=32, validation_split=0.2)

#evaluate the performance of the model
scores = model.evaluate(image_dest, depth_dest)
print("%s: %.2f%%" % (model.metrics_names[1], scores[1]*100))

if save_model:
    model.save("model_depth_nan.h5")
