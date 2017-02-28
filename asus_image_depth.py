#!/usr/bin/env python
""" Takes image and depth data from an asus xtion and saves them into a file
    named "image_depth_data.npz"
    """
import rospy

from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs import point_cloud2
import cv2
from cv_bridge import CvBridge, CvBridgeError
from visualization_msgs.msg import Marker
import numpy as np
from message_filters import ApproximateTimeSynchronizer
import message_filters


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
        self.cv_bridge = CvBridge()
        self.image_arrays = np.empty( (1, 480, 640,3) )
        self.depth_arrays = np.empty( (1, 480, 640) )

        # Unfortunately the depth data and image data from the kinect aren't
        # perfectly time-synchronized.  The code below handles this issue.
        img_sub = message_filters.Subscriber('/camera/rgb/image_raw', Image)
        cloud_sub = message_filters.Subscriber(\
                            '/camera/depth/image_raw',
                            Image)
        self.kinect_synch = ApproximateTimeSynchronizer([img_sub, cloud_sub],
                                                        queue_size=10,slop=.01)

        self.kinect_synch.registerCallback(self.image_points_callback)


        rospy.spin()

		# code for saving images and depth data using numpy.savez_compressed
        np.savez_compressed('image_depth_data',  self.image_arrays[1:,:,:,:], self.depth_arrays[1:,:,:])

    def image_points_callback(self, img, depth):
        """ Handle image/point_cloud callbacks. """

        # Convert the image message to a cv image object

        rgb_img = self.cv_bridge.imgmsg_to_cv2(img, "bgr8")
        self.image_arrays = np.append(self.image_arrays, np.resize(rgb_img, (1, 480, 640,3)), axis=0)
	depth_img = self.cv_bridge.imgmsg_to_cv2(depth)
        self.depth_arrays = np.append(self.depth_arrays, np.resize(depth_img, (1, 480, 640)), axis=0)

        print depth_img.shape, rgb_img.shape





if __name__ == "__main__":
	r = RedDepthNode()
