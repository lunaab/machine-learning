import numpy as np
import cv2
import scipy.misc as sci
import scipy.ndimage
import h5py as h

#import data from dataset here

data = h.File("../image_data/nyu_depth_v2_labeled.mat")
variables = data.items()

#extract data from the Dataset .mat
images = data.get('images')
depths  = data.get('depths')
print "***********************\n", np.max(depths), np.min(depths), depths.dtype
#print np.array(images[-1]).shape

#sci.imsave("images/actual_image.jpg", images[-1])

image_dest = np.empty((images.shape[0], 320, 240, 3))
depth_resize = np.empty((depths.shape[0], 80, 60))
depth_dest = np.empty((depths.shape[0], 80*60))

images = np.swapaxes(np.swapaxes(images, 1,3), 1,2)

#sci.imsave("images/after_swap.jpg", images[-1])

#Resize the data to fit the desired input shape (None, 320, 240, 3)
for i in range(images.shape[0]):
    image_dest[i,...] = sci.imresize(images[i,:,:,:], (320,240,3))

#sci.imsave("images/after_resize.jpg",image_dest[-1])


#Normalize the image data by the maximum pixel size
image_dest = image_dest/255

#sci.imsave("images/before_resize.jpg", depths[-1].reshape()

for d in range(depths.shape[0]):
    depth_resize[d,...] = sci.imresize(depths[d,:,:], (80,60), mode="F")
    depth_dest[d,...] = depth_resize[d].flatten()

#depth_dest = np.expand_dims(depth_dest, axis=3)

print depth_dest.shape

np.save('ny_depth', depth_dest)
np.save('ny_image', image_dest)
