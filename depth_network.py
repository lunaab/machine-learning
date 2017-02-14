from keras.models import Sequential
from keras.layers import Dense
from keras.layers import Convolution2D
from keras.layers.pooling import MaxPooling2D
from keras.layers.core import Flatten

import numpy as np
import cv2
import scipy.misc as sci
import h5py as h

#import data from dataset here

data = h.File("../image_data/nyu_depth_v2_labeled.mat")
variables = data.items()

#extract data from the Dataset .mat
images = data.get('images')
depths  = data.get('depths')

image_dest = np.empty((images.shape[0], 320, 240, 3))
depth_resize = np.empty((depths.shape[0], 80, 60, 1))
depth_dest = np.empty((depths.shape[0], 80*60))

depths = np.expand_dims(depths, axis=3)
images = np.swapaxes(np.swapaxes(images, 1,3), 1,2)

#Resize the data to fit the desired input shape (None, 320, 240, 3)
for i in range(images.shape[0]):
    image_dest[i,...] = sci.imresize(images[i,:,:,:], (320,240,3))

#Normalize the image data by the maximum pixel size
image_dest = image_dest/255

for d in range(depths.shape[0]):
    depth_resize[d,...] = np.resize(depths[d,:,:,:], (80,60,1))
    depth_dest[d,...] = depth_resize[d].flatten()

#here the model architecture is defined
#using TS for background so depth data has to be ordered ()
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


#Coarse 5 (?? max pooling? Really?)
model.add(Convolution2D(256,3,3,activation='relu',dim_ordering='tf'))#,input_shape=(20,15,3)
model.add(MaxPooling2D(pool_size=(2,2)))

#Coarse 6
model.add(Flatten())
model.add(Dense(4096, init='uniform', activation='relu'))
#model.add(Dense(80*60, init='uniform', activation='relu'))


model.add(Dense(80*60, init='uniform', activation='linear'))

model.compile(loss='mean_squared_error', optimizer='RMSprop', metrics = ['accuracy'])
model.fit(image_dest, depth_dest, nb_epoch=10, batch_size=100, validation_split=0.2)

scores = model.evaluate(image_dest, depth_dest)
print("%s: %.2f%%" % (model.metrics_names[1], scores[1]*100))
