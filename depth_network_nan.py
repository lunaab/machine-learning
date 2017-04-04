from keras.models import Sequential
from keras.layers import Dense
from keras.layers import Convolution2D
from keras.layers.pooling import MaxPooling2D
from keras.layers.core import Flatten
from keras.layers.core import Dropout

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

def nan_mse(output, target):
     sub = tf.subtract(output, target)
     sqr = tf.square(sub)
     nans = tf.logical_not(tf.is_nan(target))
     nans_to_zero = tf.cast(nans, tf.float32)
     big_num = tf.constant(10000000.0)
     nans_to_zero_and_big = tf.multiply(nans_to_zero, big_num)
     sqr = tf.minimum(sqr, nans_to_zero_and_big)
     num_non_nans = tf.reduce_sum(nans_to_zero, axis=-1)
     return tf.div(tf.reduce_sum(sqr, axis=-1), num_non_nans)

#here the model architecture is defined
#using TS for background so depth data has to be ordered (width, heighth, depth)
model = Sequential()

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
model.compile(loss=nan_mse, optimizer='adam', metrics = ['accuracy'])

#fit the model to matching depth data
model.fit(image_dest, depth_dest, nb_epoch=60, batch_size=32, validation_split=0.2)

#evaluate the performance of the model
scores = model.evaluate(image_dest, depth_dest)
print("%s: %.2f%%" % (model.metrics_names[1], scores[1]*100))

if save_model:
    model.save("model_depth.h5")
