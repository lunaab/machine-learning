#!/usr/bin/env python
""" Takes image and depth data from an asus xtion and saves them into a file
    named "image_depth_data.npz"
    """
import rospy
import serial

from rospy.numpy_msg import numpy_msg
from accel_pub_sub.msg import Floats
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
        
        """ Construct the red-pixel finder node."""
        rospy.init_node('red_depth_node')
        self.save_loc = "data_collect/" + sys.argv[1]
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
        motion_sub = message_filters.Subscriber('accel_data', numpy_msg(Floats))
        print 'prepare to synch'
        self.kinect_synch = ApproximateTimeSynchronizer([img_sub, cloud_sub, motion_sub],
                                                        queue_size=10,slop=.02)
        print 'prepare callback'
        self.kinect_synch.registerCallback(self.image_points_callback)
        
        
        rospy.spin()

		# code for saving images and depth data using numpy.savez_compressed
        np.savez_compressed(self.save_loc,  self.image_arrays[1:,:,:,:], self.depth_arrays[1:,:,:], self.motion_arrays[1:,:,:])

    def image_points_callback(self, img, depth, motion):
        """ Handle image/point_cloud callbacks. """

        # Convert the image message to a cv image object
        print 'inside callback'
        rgb_img = self.cv_bridge.imgmsg_to_cv2(img, "bgr8")
        depth_img = self.cv_bridge.imgmsg_to_cv2(depth)
        motion_img = np.zeros( (1, 3, 2) )
        rospy.loginfo(motion.data)
        print 'appending'
        motion_img[0][0][0] = motion.data[0]
        motion_img[0][0][1] = motion.data[1]
        motion_img[0][1][0] = motion.data[2]
        motion_img[0][1][1] = motion.data[3]
        motion_img[0][2][0] = motion.data[4]
        motion_img[0][2][1] = motion.data[5]
        
        
        self.motion_arrays = np.append(self.motion_arrays, np.resize(motion_img, (1, 3, 2)), axis=0)
        self.image_arrays = np.append(self.image_arrays, np.resize(rgb_img, (1, 480, 640,3)), axis=0)
        self.depth_arrays = np.append(self.depth_arrays, np.resize(depth_img, (1, 480, 640)), axis=0)

        print depth_img.shape, rgb_img.shape


        


if __name__ == "__main__":
	if len(sys.argv) != 2:
                print "Usage: \"python asus_image_depth.py <destination_no_extension>\""
                exit(-1)
        r = RedDepthNode()
