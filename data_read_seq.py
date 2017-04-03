import numpy as np
import cv2
import scipy.misc as sci
import scipy.ndimage
import h5py as h
import os
import sys

class DataRead():
    def __init__(self):
        self.images = np.empty((0,640,480,3))
        self.depths = np.empty((0,640,480))
        self.accel  = np.empty((0,3,2))

    def dataRead(self, dir):
        if not os.path.isdir(dir):
            print "Not a directory. Give a Directory name."
            exit(-1)
        i = 0
        for f in os.listdir(dir):
            f = "data_collect/" + f
            if not os.path.isdir(f):
                data = np.load(f)
                if i == 0:
                    id = data['arr_0']
                    dd = data['arr_1']
                    ad = data['arr_2']
                else:
                    np.append(id,data['arr_0'], axis=0)
                    np.append(dd,data['arr_1'], axis=0)
                    np.append(ad,data['arr_2'], axis=0)
             i += 1

        print id.shape
        print dd.shape
        print ad.shape

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print "Usage: \"python data_read_seq.py <data directory name>\""
        exit(-1)
    d = DataRead()
    d.dataRead(sys.argv[1])


"""
#import data from dataset here

data = h.File("../image_data/nyu_depth_v2_labeled.mat")
variables = data.items()

#extract data from the Dataset .mat
images = data.get('images')
depths  = data.get('depths')
print "***********************\n"
print "****Reading in data****"

image_dest = np.empty((images.shape[0], 320, 240, 3))
depth_resize = np.empty((depths.shape[0], 5, 5))
depth_dest = np.empty((depths.shape[0], 5*5))

images = np.swapaxes(np.swapaxes(images, 1,3), 1,2)

random_selections = np.random.choice(images.shape[0], images.shape[0], replace=False)

print "***********************\n"
print "****Resizing Images****"
#Resize the data to fit the desired input shape (None, 320, 240, 3)
i = 0
for r in random_selections:
    image_dest[i,...] = sci.imresize(images[r,:,:,:], (320,240,3))
    i += 1

#print random_selections.shape
#print max(random_selections)

#Normalize the image data by the maximum pixel size
image_dest = image_dest/255

#sci.imsave("images/before_resize.jpg", depths[-1].reshape()

print "***********************\n"
print "****Resizing Depths****"
i = 0
for r in random_selections:
    depth_resize[i,...] = sci.imresize(depths[r,:,:], (5,5), mode="F")
    depth_dest[i,...] = depth_resize[i].flatten()
    i += 1

#depth_dest = np.expand_dims(depth_dest, axis=3)

print "Depth Shape: ", depth_dest.shape

print "***********************\n"
print "******Saving Data******"
np.save('ny_depth', depth_dest)
np.save('ny_image', image_dest)
print "Done"

"""
