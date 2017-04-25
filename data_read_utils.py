import numpy as np
import cv2
import scipy.misc as sci
import scipy.ndimage
import h5py as h
import os

"""
data_read_utils supplies the necessary functions
for reading in image, depth, and accelerometer data
for the purposes of The Depth Perception System(JMU 2017)

Author: Nathan Johnson
"""


"""
read_from_mat takes in an ".h5" file (matlab)
that is compressed with a specific set of
objects. The file used in this case is called:
  nyu_depth_v2_labeled.mat

A full description of this dataset can be found at:

  http://cs.nyu.edu/~silberman/datasets/nyu_depth_v2.html

This method reads the depth and image data into two npys:

  ny_depth with shape (,5,5)
  ny_image with shape (,320,240,3)

Author: Nathan Johnson
"""
def read_from_mat(filename):
    if not os.path.exists(filename):
        print "File does not exist. Try Again."
        exit(-1)

    #import data from dataset here
    data = h.File(filename)
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

Author: Nathan Johnson
"""
def read_from_dir(dir, file_prefix):
    debug = False
    save_dir = 'io_net_data/'

    if not os.path.isdir(dir):
        print "Not a directory. Give a Directory name."
        exit(-1)
    i = 0
    for f in os.listdir(dir):
        f = dir + "/" + f
        if not os.path.isdir(f):
            data = np.load(f)
            if i == 0:
                i += 1
                id = data['arr_0']
                dd = data['arr_1']
                ad = data['arr_2']
            else:
                id = np.append(id,data['arr_0'], axis=0)
                dd = np.append(dd,data['arr_1'], axis=0)
                ad = np.append(ad,data['arr_2'], axis=0)
                a = id.shape[0] - dd.shape[0]
                if a > 0:
                    for p in range(a):
                        id = np.delete(id, -1, axis=0)

    if debug:
        print id.shape

    image_dest = np.empty((id.shape[0], 320, 240, 3))
    depth_dest = np.empty((dd.shape[0], 5*5))

    random_selections = np.random.choice(id.shape[0], id.shape[0], replace=False)

    id = np.swapaxes(np.swapaxes(id, 1,3), 1,2)

    i = 0
    for r in random_selections:
        image_dest[i,...] = sci.imresize(id[r,:,:,:], (320,240,3))
        i += 1

    d_min = np.empty((5,5))
    d_mins = np.empty((5,5))

    bool = True
    for d in dd:
        for i in range(0,5):
            for j in range(0,5):
                sec_mins = [d_val for d_val in d[i*96:(i + 1)*96,j*128:(j+1)*128].flatten() if d_val > 0]
                if len(sec_mins) == 0:
                    d_min[i, j] = 0
                else:
                    d_min[i,j] = min(sec_mins)
        if bool:
            d_mins = [d_min]
            bool = False
        else:
            d_mins = np.concatenate((d_mins, [d_min]), axis=0)

    if debug:
        print d_mins.shape
        print d_mins

    i = 0
    for r in random_selections:
        depth_dest[i,...] = d_mins[i].flatten()
        i += 1

    image_dest = image_dest/255
    depth_dest = depth_dest/1000

    depth_dest[depth_dest == 0] = float('nan')

    if debug:
        print "Image shape: ", image_dest.shape
        print "Depth shape: ", depth_dest.shape
        print "Max depth value: ", max(depth_dest[0])

    np.save(save_dir + file_prefix + '_image_data.npy',image_dest)
    np.save(save_dir + file_prefix + '_depth_data.npy',depth_dest)
    np.save(save_dir + file_prefix + '_accel_data.npy',ad)


"""
convert_im_dir takes in a directory name and
converts all of the float values in images to uint_8
since all rgb values are 0-255.
This drastically cuts down on the size of the files.

Author: Nathan Johnson
"""
def convert_im_dir(dir):
    if not os.path.isdir(dir):
        print "Not a directory. Give a Directory name."
        exit(-1)

    for f in os.listdir(dir):
        f = dir + "/" + f
        if not os.path.isdir(f):
            data = np.load(f)
            i = data['arr_0']
            asint = i.astype('uint8')
            np.savez_compressed(f, asint, data['arr_1'], data['arr_2'])
