#!/usr/bin/env python

import rospy

from mil_msgs.msg import ObjectsInImage, ObjectInImage
from mil_vision_tools import CentroidWidthHeightTracker

from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from overlay import Overlay


class image_object_tracker:
    def __init__(self):
        
        #Subscribers
        
        rospy.Subscriber(rospy.get_param("camera_feed"), Image, self.image_cb)
        
        rospy.Subscriber('VISION', ObjectsInImage,self.objects_in_image_cb)
        
        self.pub = rospy.Publisher('persistent_objects_in_image', ObjectsInImage, queue_size = 1)
        
        
        
        self.tracker = CentroidWidthHeightTracker(max_distance=rospy.get_param("max_distance"))
        
    def image_cb(self, msg):
        
        persistent = self.tracker.get_persistent_objects(min_observations=8, min_age=rospy.Duration(0))
        objects_in_image = ObjectsInImage()
        objects_in_image.header = msg.header
        objects_in_image.objects = [i.data for i in persistent]
        self.pub.publish(objects_in_image)
        
        
    def objects_in_image_cb(self, objects_in_image):
        self.tracker.clear_expired(now=objects_in_image.header.stamp)
        
        for i in objects_in_image.objects:
            obj = self.tracker.add_observation(objects_in_image.header.stamp, np.array(self.features(i)), data=i)
            
            
    def features(self, object_in_image):
        
        centroid = self.centroid(object_in_image)
        width_height = self.width_height(object_in_image)
        
        return [centroid[0],centroid[1],width_height[0], width_height[1]]
        
        
    
    def centroid(self, object_in_image):
        x=0
        y=0
        if len(object_in_image.points) != 0:
            n=0
            for i in object_in_image.points:
                x+=i.x
                y+=i.y
                n+=1
            x = int(x/n)
            y = int(y/n)
        return [x,y]
    
    
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





