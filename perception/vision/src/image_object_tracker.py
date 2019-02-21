#!/usr/bin/env python

import rospy

from mil_msgs.msg import ObjectsInImage, ObjectInImage
from mil_vision_tools import CentroidObjectsTracker

from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from overlay import Overlay

#currently tracks objects with centroid...should use something better or be configurable
class image_object_tracker:
    def __init__(self):
        
        #Subscribers
        
        rospy.Subscriber("/camera/front/right/image_raw", Image, self.image_cb)
        
        rospy.Subscriber('VISION', ObjectsInImage,self.objects_in_image_cb)
        
        self.pub = rospy.Publisher('tracked_objects', ObjectsInImage, queue_size = 1)
        
        self.tracker = CentroidObjectsTracker(max_distance=20.0)
        
    def image_cb(self, msg):
        persistent = self.tracker.get_persistent_objects(min_observations=8, min_age=rospy.Duration(0))
        objects_in_image = ObjectsInImage()
        objects_in_image.header = msg.header
        objects_in_image.objects = [i.data.object for i in persistent] 
        self.pub.publish(objects_in_image)
        
        
    def objects_in_image_cb(self, objects_in_image):
        self.tracker.clear_expired(now=objects_in_image.header.stamp)
        
        for i in objects_in_image.objects:
            o = Overlay(header = objects_in_image.header,object_in_image = i)
            obj = self.tracker.add_observation(objects_in_image.header.stamp, np.array(o.centroid()), data=o)
    
if __name__ == '__main__':
    rospy.init_node('image_object_tracker', anonymous = False)
    image_object_tracker = image_object_tracker()
    rospy.spin()





