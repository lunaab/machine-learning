#!/usr/bin/env python

import rospy
import serial
import numpy
import time
from rospy.numpy_msg import numpy_msg
from accel_pub_sub.msg import Floats

def accel_pub():
  pub = rospy.Publisher('accel_data', numpy_msg(Floats), queue_size=1000)
  rospy.init_node('accel_data', anonymous=False)
  rate = rospy.Rate(60)


  rospy.loginfo('Connecting to serial')
  ser = serial.Serial('/dev/ttyUSB1', 9600)
  rospy.loginfo('Connected to serial')
  time.sleep(2)
  while not rospy.is_shutdown():
    msg = Floats()
    accel_arr = numpy.zeros( (6,) )
    rospy.loginfo('Reading from serial')
    #ser.reset_input_buffer()
    ser_inc = ser.readline()
    msg.header.stamp = rospy.get_rostime()    
    ser_data = ser_inc.split(" ")
    rospy.loginfo('Read from serial')
    rospy.loginfo(ser_data[0])
    rospy.loginfo(ser_data[1])
    rospy.loginfo(ser_data[2])
    rospy.loginfo(ser_data[3])
    rospy.loginfo(ser_data[4])
    rospy.loginfo(ser_data[5].translate(None, '\n'))
    rospy.loginfo(ser_data)
    accel_arr[0] = float(ser_data[0])
    accel_arr[1] = float(ser_data[1])
    accel_arr[2] = float(ser_data[2])
    accel_arr[3] = float(ser_data[3])
    accel_arr[4] = float(ser_data[4])
    accel_arr[5] = float(ser_data[5].translate(None, '\n'))
    
    msg.data = accel_arr
    rospy.loginfo(msg)
    pub.publish(msg)
    rate.sleep()
    
if __name__ == '__main__':
  accel_pub()

	

