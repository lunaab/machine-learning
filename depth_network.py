from keras.models import Sequential
from keras.layers import Dense
from keras.layers import Convolution2D
from keras.layers.pooling import MaxPooling2D
from keras.layers.core import Flatten
from keras.layers.core import Dropout
from keras.optimizers import SGD
from keras import backend as K

from matplotlib import pyplot as plt
import tensorflow as tf
import numpy as np
import cv2
import scipy.misc as sci
import h5py as h
import os.path
import network_utils as nu


"""
DepthNetwork defines the structure of a Convolutional Neural Network
that predicts depths from rgb images.

Author: Nathan Johnson
"""
class DepthNetwork():
     def __init__(self, images, depths, epochs=10, loss='mean_squared_error', save_model=True):
         self.loss = loss
         self.save_model = save_model
         self.epochs = epochs
         if os.path.isfile(images) and os.path.isfile(depths):
             self.image_dest = np.load(images)
             self.depth_dest = np.load(depths)
         else:
             print "files do not exist. Exiting."
             exit(-1)

         #here the model architecture is defined
         #using TS for background so depth data has to be ordered (width, heighth, depth)
         self.model = self.define_model(loss)

     """
     The structure of the model is defined here
     """
     def define_model(self, loss='mean_squared_error', optimizer='adam'):
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
         model.compile(loss=loss, optimizer=optimizer)

         return model


     """
     fit does the training. Also saves the model if desired
     """
     def fit(self, save_name="model_depth.h5"):
         self.model.fit(self.image_dest, self.depth_dest, nb_epoch=self.epochs, batch_size=32, validation_split=0.2)

         if self.save_model:
             self.model.save(save_name)

     """
     show_predictions takes in a set of images as an numpy array and iterates through
     to perform a depth prediction on each image. The each image and
     depth pair are shown together as a plot.

     @param: images numpy array of shape (,n,m,3)
     """
     def show_predictions(self, images):

         for i in range(images.shape[0]):
             #make prediction
             depth_pre = self.model.predict(np.array(images[i,:,:,:]).reshape((1,320,240,3)), batch_size=1)
             depth_pre = np.resize(depth_pre,(5,5))

             fig, ax = plt.subplots()

             #create colorbar
             cax = ax.imshow(depth_pre, interpolation='none')
             fig.colorbar(cax)

             plt.subplots()
             plt.imshow(images[i,:,:,:])

             print depth_pre
             plt.show()

             cont = raw_input( "continue? (y/n): ")
             if cont != "y":
                 break

if __name__ == '__main__':
    network = DepthNetwork('io_net_data/image_data.npy','io_net_data/depth_data.npy', epochs=10, loss=nu.mse_ignore_nan)
    network.fit()
    #network.show_predictions(np.load('io_net_data/image_data.npy'))
