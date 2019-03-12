#!/usr/bin/env python
#TODO add smart filter adjuster

import rospy

import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import ast

from mil_vision_tools import CentroidObjectsTracker, TrackedObject
from vision_utils import centroid
from overlay import Overlay
from mil_msgs.msg import ObjectsInImage, ObjectInImage
import numpy as np

class image_object_tracker:
    def __init__(self):
        
        #Subscribers
        
        rospy.Subscriber(rospy.get_param("camera_feed"), Image, self.image_cb)
        
        rospy.Subscriber('VISION', ObjectsInImage,self.objects_in_image_cb)
        
        self.pub_objects_in_image = rospy.Publisher('persistent_objects_in_image', ObjectsInImage, queue_size = 1)
        
        self.tracker = CentroidObjectsTracker(expiration_seconds = rospy.get_param("expiration_seconds"), max_distance=rospy.get_param("max_distance"))
        
        
        
    def image_cb(self, msg):
        self.tracker.clear_expired(now=msg.header.stamp)
        persistent = self.tracker.get_persistent_objects(min_observations=rospy.get_param("min_observations"), min_age=rospy.Duration(0))
        objects_in_image = ObjectsInImage()
        objects_in_image.header = msg.header
        for i in persistent:
            temp = ast.literal_eval(i.data.attributes)
            temp["id"] = i.id
            i.data.attributes = str(temp)
            objects_in_image.objects.append(i.data)
        self.pub_objects_in_image.publish(objects_in_image)
        
        
    def objects_in_image_cb(self, objects_in_image):
        self.tracker.clear_expired(now=objects_in_image.header.stamp)
        
        for i in objects_in_image.objects:
            c = centroid(i)
            i.attributes = str({"centroid": c})
            obj = self.tracker.add_observation(objects_in_image.header.stamp, np.array(c), data=i)
            
            
    def features(self, object_in_image):
        
        c = centroid(object_in_image)
        return [c[0],c[1]]
        
    
    def width_height(self, object_in_image):
        
        if len(object_in_image.points) ==0:
            return [0,0]
        
        minX = object_in_image.points[0].x
        maxX = object_in_image.points[0].x
        
        minY = object_in_image.points[0].y
        maxY = object_in_image.points[0].y
        
        for i in object_in_image.points:
            if i.x>maxX:
                maxX = i.x
            if i.x<minX:
                minX = i.x
                
            if i.y>maxY:
                maxY = i.x
            if i.y<minY:
                minY = i.x
            
        return [maxX-minX, maxY-minY]


if __name__ == '__main__':
    rospy.init_node('image_object_tracker', anonymous = False)
    image_object_tracker = image_object_tracker()
    rospy.spin()





