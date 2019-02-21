#!/usr/bin/env python

import rospy

from mil_msgs.msg import ObjectsInImage, ObjectInImage
from mil_vision_tools import CentroidObjectsTracker

from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from numpy import uint8
from feature import Feature

#currently tracks objects with centroid...should use something better or be configurable
class image_object_tracker:
    def __init__(self):
        
        #Subscribers
        
        #i = Overlay(header = None, object_in_image = None)
        
        rospy.Subscriber("/camera/front/right/image_raw", Image, self.image_cb)
        
        rospy.Subscriber('VISION', ObjectsInImage,self.objs_cb)
        
        self.pub = rospy.Publisher('tracked_overlays', Feature, queue_size = 1)
        
        self.tracker = CentroidObjectsTracker(max_distance=20.0)
        
        self.persistent=[]
        
        
    def image_cb(self, msg):
         self.persistent = self.tracker.get_persistent_objects(min_observations=8, min_age=rospy.Duration(0))
        
         self.pub.publish(self.persistent)
        
        
    def objs_cb(self, objs):
        self.tracker.clear_expired(now = objs.header.stamp)
        
        for i in objs.objects:
            o = Overlay(header = objs.header,object_in_image = i)
            obj = self.tracker.add_observation(objs.header.stamp, np.array(o.centroid()), data=o)

    
if __name__ == '__main__':
    rospy.init_node('image_object_tracker', anonymous = False)
    image_object_tracker = image_object_tracker()
    rospy.spin()





