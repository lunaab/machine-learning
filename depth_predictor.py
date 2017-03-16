#!/usr/bin/env/ python

import rospy
import numpy as np
import scipy.misc as sci
import h5py as h
import sys
import os.path
import cv2
import message_filters

from cv_bridge import CvBridge, CvBridgeError
from keras.models import load_model
from keras.models import Sequential

class DepthPredictor (object):
    """
    This node subcribes to a camera topic and uses the images
    passed to the callback to predict depths using a pretrained
    model.
    Subscribes:
        /camera/rgb/image_color
    Publishes:
        TODO
    """
    def __init__(self):
        """ Read in the model. Start the node. Set up Subs and Pubs"""
        rospy.init_node('depth_predictor')
        
        self.model = load_model("model_name.h5")

        rospy.spin()

    def image_prediction_callback(self, img):
        """
        This method recieves an image and converts in to an np array.
        The image is then given to the model for an prediction output.
        The prediction is then published to a topic to be processed.

        img = np.array(sci.imresize(np.array(img), (320,240,3)))
        prediction = self.model.predict(img, batch_size=1, verbose=1)
        publisher.pub(prediction)
        """
        pass

if __name__ == "__main__":
    d = DepthPredictor()
