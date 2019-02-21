#!/usr/bin/env python

import rospy

from mil_msgs.msg import ObjectsInImage
from mil_msgs.msg import ObjectInImage
#from mil_msgs import Point2D
from mil_vision_tools import CentroidObjectsTracker

from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from numpy import uint8
from overlay import Overlay
from random import randint
import sys
import ast

class visualization:
    def __init__(self, disco = False, minus = [], only = [],**kwargs):
        
        assert not (only != [] and minus !=[])
        
        #Subscribers
        rospy.Subscriber(rospy.get_param("camera_feed"), Image,self.image_cb)
        
        rospy.Subscriber('tracked_objects', ObjectsInImage, self.tracked_objects_cb)
        
        self.image = Image()
        self.overlays = []
        self.bridge = CvBridge()
        
        self.disco = disco
        
        self.colors = self.color_gen(64)
        
        self.minus = minus
        self.only = only
        
    def tracked_objects_cb(self, tracked_objects):
        
        if tracked_objects.header.stamp == self.image.header.stamp:
            self.overlays = []
            self.overlays = [Overlay(header = tracked_objects.header, object_in_image = i) for i in tracked_objects.objects]
        
    def image_cb(self, msg):
        
        self.image = msg
        
        overlays = self.select_overlays(msg, self.overlays)
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        cv_image = self.draw_on(cv_image, overlays)
        
        cv2.imshow('image',cv_image)
        cv2.waitKey(1)
    
    def draw_on(self, img, overlays=[]):
        for i in overlays:
            img = i.draw_on(img)
        return img
    
    def select_overlays(self,msg, all_overlays=[]):
        
        selected_overlays = all_overlays
        
        if self.minus!=[]:
            for i in all_overlays:
                if (i.object.name in self.minus):
                    selected_overlays.remove(i)
        
        elif self.only !=[]:
            for i in all_overlays:
                if (i.object.name not in self.only):
                    selected_overlays.remove(i)
        
        selected_overlays.sort(key = lambda x: x.object.name)
        
        #the overlays are also given non-default colros here
        
        if self.disco == True:
            self.colors = []
            self.colors = self.color_gen(len(selected_overlays))
        i =0
        for j in selected_overlays:
            j.color = self.colors[i]
            i+=1
            if i>=len(self.colors):
                i=0
        return selected_overlays
    
    def color_gen(self, n):
        c = []
        for i in range(n):
            c.append((randint(0,255),randint(0,255),randint(0,255)))
        return c



if __name__ == '__main__':
    rospy.init_node('visualization', anonymous=False)
    
    kwargs = {}
    
    kwargs.update({'disco':rospy.get_param("disco")})
    
    kwargs.update({'minus':ast.literal_eval(rospy.get_param("minus"))})
    
    kwargs.update({'only':ast.literal_eval(rospy.get_param("only"))})
    
    
    visualization = visualization(**kwargs)
    rospy.spin()
    cv2.destroyAllWindows()

