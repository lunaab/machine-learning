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

def mse_ignore_nan(y_true, y_pred):
     ok_entries = tf.logical_not(tf.is_nan(y_true))
     safe_targets = tf.where(ok_entries, y_true, y_pred)
     sqr = tf.square(y_pred - safe_targets)

     zero_nans = tf.cast(ok_entries, K.floatx())
     num_ok = tf.reduce_sum(zero_nans, axis=-1) # count OK entries
     num_ok = tf.maximum(num_ok, tf.ones_like(num_ok)) # avoid divide by zero
     return tf.reduce_sum(sqr, axis=-1) / num_ok

def define_model(loss):
    #here the model architecture is defined
    #using TS for background so depth data has to be ordered (width, heighth, depth)
    model = Sequential()

    #sgd = SGD(lr=0.01, decay=1e-6, momentum=0.9)

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
    model.compile(loss=loss, optimizer='adam')

    return model
