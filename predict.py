from keras.models import Sequential
from keras.models import model_from_yaml
from keras.layers import Dense
from keras.layers import Convolution2D
from keras.layers.pooling import MaxPooling2D
from keras.layers.core import Flatten
from keras.models import load_model

import numpy as np
import cv2
import scipy.misc as sci
import h5py as h
import os.path

model = load_model("depth_net.h5")

print "Model has been loaded\n"
