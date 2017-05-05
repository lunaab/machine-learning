#!/usr/bin/env python
""" Takes image and depth data from an asus xtion and saves them into a file
    named "image_depth_data.npz"
    """
import rospy
import serial


from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs import point_cloud2
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import message_filters



class RedDepthNode(object):
    """ This node reads from the Kinect color stream and
    publishes an image topic with the most red pixel marked.
    Subscribes:
         /camera/rgb/image_color
    Publishes:
         red_marked_image
    """

    def __init__(self):
        
        """ Construct the red-pixel finder node."""
        rospy.init_node('red_depth_node')
        self.arduino = serial.Serial('/dev/ttyUSB0', 19200)
        self.cv_bridge = CvBridge()
        self.depth_arrays = np.empty( (1, 480, 640) )
        self.farest_trigger = 1000.0
        self.lowest_trigger = 400
        self.slice_width = 0.02
        self.highV = 155.0
        self.lowV = 10
        self.slope = (self.lowV - self.highV) / (self.farest_trigger - self.lowest_trigger)

        # Unfortunately the depth data and image data from the kinect aren't
        # perfectly time-synchronized.  The code below handles this issue.
        self.cloud_sub = message_filters.Subscriber(\
                            '/camera/depth/image_raw',
                            Image)
        rospy.loginfo('Registering Callback')
        self.cloud_sub.registerCallback(self.image_points_callback)
        
        
        rospy.spin()


    def image_points_callback(self, depth):
        """ Handle image/point_cloud callbacks. """

        # Convert the image message to a cv image object
        rospy.loginfo('Inside Callback')
        depth_img = self.cv_bridge.imgmsg_to_cv2(depth)
        
        (row, col) = depth_img.shape
        spacing = col / 5
        #width = (int)(self.slice_width * row)
        #width_start = (row - width) / 2
        #width_end = width_start + width
        #rospy.loginfo(width)
        #rospy.loginfo(width_start)
        
        #print self.slope
        depths = depth_img[depth_img.shape[0]/2, 0::spacing]
        print depths
        depths = (self.slope * depths) - (self.slope * self.lowest_trigger) + self.highV
        depths = depths.astype(int)
        print depths
        for i in range(0,5):
            if (depths[i] > self.highV):
                depths[i] = self.highV
            if (depths[i] < self.lowV):
                depths[i] = 0
        print depths
        toSend = "a,%d,b,%d,c,%d,d,%d,e,%d," % (depths[0], depths[1], depths[2], depths[3], depths[4])
        self.arduino.write(toSend) 


if __name__ == "__main__":
	r = RedDepthNode()
