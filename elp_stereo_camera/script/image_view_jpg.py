#! /usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import sys, time
import roslib
import rospy
import numpy as np

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

def callback(data):
    im = cv2.imdecode(data.data, CV_LOAD_IMAGE_COLOR)
    cv2.imshow("iphone",im) 
    cv2.waitKey(0);
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/image_jpg", CompressedImage, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
