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

np.save('ny_depth', depth_dest)
np.save('ny_image', image_dest)
