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
from visualization_msgs.msg import Marker
import numpy as np
from message_filters import ApproximateTimeSynchronizer
import message_filters
import sys


def make_sphere_marker(x, y, z, scale, frame_id, stamp):
    """ Create a red sphere marker message at the indicated position. """
    m = Marker()
    m.header.stamp = stamp
    m.header.frame_id = frame_id
    m.ns = 'spheres'
    m.id = 0
    m.type = m.SPHERE
    m.action = m.ADD
    m.pose.position.x = x
    m.pose.position.y = y
    m.pose.position.z = z
    m.scale.x = m.scale.y = m.scale.z =  scale    
    m.color.r = 1.0
    m.color.a = 1.0
    return m

class RedDepthNode(object):
    """ This node reads from the Kinect color stream and
    publishes an image topic with the most red pixel marked.
    Subscribes:
         /camera/rgb/image_color
    Publishes:
         red_marked_image
    """

    def __init__(self):
        self.save_loc = "data_collect/" + sys.argv[1] + ".npz"
        self.ser = serial.Serial('/dev/ttyUSB0')
        self.ser.baudrate = 9600
        """ Construct the red-pixel finder node."""
        rospy.init_node('red_depth_node')
        self.cv_bridge = CvBridge()
        self.image_arrays = np.empty( (1, 480, 640,3) )
        self.depth_arrays = np.empty( (1, 480, 640) )
        self.motion_arrays = np.empty( (1, 3, 2) )

        # Unfortunately the depth data and image data from the kinect aren't
        # perfectly time-synchronized.  The code below handles this issue.
        img_sub = message_filters.Subscriber('/camera/rgb/image_raw', Image)
        cloud_sub = message_filters.Subscriber(\
                            '/camera/depth/image_raw',
                            Image)
        self.kinect_synch = ApproximateTimeSynchronizer([img_sub, cloud_sub],
                                                        queue_size=10,slop=.01)

        self.kinect_synch.registerCallback(self.image_points_callback)
        
        #rospy.init_node(
        #rospy.Subscriber('chatter', Float64MultiArray, self.accel_call 


        rospy.spin()

		# code for saving images and depth data using numpy.savez_compressed
        np.savez_compressed(self.save_loc,  self.image_arrays[1:,:,:,:], self.depth_arrays[1:,:,:], self.motion_arrays[1:,:,:])

    def image_points_callback(self, img, depth):
        """ Handle image/point_cloud callbacks. """

        # Convert the image message to a cv image object
        print "in call back\n"
        rgb_img = self.cv_bridge.imgmsg_to_cv2(img, "bgr8")
        depth_img = self.cv_bridge.imgmsg_to_cv2(depth)
        #print "recieved depth and rgd images"
  
        self.ser.write('f')
        motion = self.ser.readline()
        motion_split = motion.split()
        if (len(motion_split) != 6):
            print "ERROR"
        else:    
            motion_sub = np.empty( (1, 3, 2) )
            motion_sub[0][0][0] = float(motion_split[0])
            motion_sub[0][0][1] = float(motion_split[1])
            motion_sub[0][1][0] = float(motion_split[2])
            motion_sub[0][1][1] = float(motion_split[3])
            motion_sub[0][2][0] = float(motion_split[4])
            motion_sub[0][2][1] = float(motion_split[5])
            self.motion_arrays = np.append(self.motion_arrays, motion_sub, axis=0)
        print ""

        self.image_arrays = np.append(self.image_arrays, np.resize(rgb_img, (1, 480, 640,3)), axis=0)
        self.depth_arrays = np.append(self.depth_arrays, np.resize(depth_img, (1, 480, 640)), axis=0)

        print depth_img.shape, rgb_img.shape


        


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print "Usage: \"python asus_image_depth.py <destination_no_extension>\""
        exit(-1)
    r = RedDepthNode()
