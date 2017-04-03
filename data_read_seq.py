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

        image_dest = np.empty((id.shape[0], 320, 240, 3))
        depth_resize = np.empty((dd.shape[0], 5, 5))
        depth_dest = np.empty((dd.shape[0], 5*5))

        random_selections = np.random.choice(id.shape[0], id.shape[0], replace=False)

        i = 0
        for r in random_selections:
            image_dest[i,...] = sci.imresize(id[r,:,:,:], (320,240,3))
            i += 1

        i = 0
        for r in random_selections:
            depth_resize[i,...] = sci.imresize(dd[r,:,:], (5,5), mode="F")
            depth_dest[i,...] = depth_resize[i].flatten()
            i += 1



        np.save('image_data.npy',id)
        np.save('depth_data.npy',dd)
        np.save('accel_data.npy',ad)

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print "Usage: \"python data_read_seq.py <data directory name>\""
        exit(-1)
    d = DataRead()
    d.dataRead(sys.argv[1])

