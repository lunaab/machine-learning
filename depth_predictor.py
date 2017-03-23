#!/usr/bin/env/ python

import rospy
import numpy as np
import scipy.misc as sci
import sys
import os.path
import cv2
import h5py as h
import message_filters

from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
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
        /depth_pre
    """
    def __init__(self):
        """ Read in the model. Start the node. Set up Subs and Pubs"""
        rospy.init_node('depth_predictor')

        self.model = load_model("../models/model_depth.h5")

        #create publisher
        self.depth_pub = rospy.Publisher('depth_pre', numpy_msg(Floats), queue_size=10)

        #Video Cap set to 1 to get USB camera plugged in.
        self.capture = cv2.VideoCapture(1)
        self.width = 320
        self.height = 240
        self.capture.set(4, self.width)
        self.capture.set(3, self.height)
        self.predict_pub()
        rospy.spin()

    def predict_pub(self):
        """
        This method recieves an image and converts in to an np array.
        The image is then given to the model for an prediction output.
        The prediction is then published to a topic to be processed.
        """
        while not rospy.is_shutdown():
            success, img = self.capture.read()
            img = np.swapaxes(img,0,1)/255
            #print img.shape
            prediction = self.model.predict(np.array([img]), batch_size=1, verbose=0)
            print prediction.shape
            self.depth_pub.publish(prediction)


if __name__ == "__main__":
    d = DepthPredictor()

