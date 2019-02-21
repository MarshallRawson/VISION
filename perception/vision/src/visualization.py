#!/usr/bin/env python

import rospy

from mil_msgs.msg import ObjectsInImage, ObjectInImage
from mil_vision_tools import CentroidObjectsTracker

from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from numpy import uint8
from overlay import Overlay
from random import randint

class visualization:
    def __init__(self, disco = False, minus = [], only = [],**kwargs):
        
        assert not (only != [] and minus !=[])
        
        #Subscribers
        
        rospy.Subscriber(rospy.get_param('camera_feed'), Image, self.image_cb)
        
        #rospy.Subscriber('VISION', ObjectsInImage,self.objects_in_image_cb)
        
        rospy.Subscriber('tracked_overlays', Overlay ,self.overlay_cb)
        
        self.objects_in_image = ObjectsInImage()
        self.overlays = []
        self.bridge = CvBridge()
        
        self.disco = disco
        
        self.colors = self.color_gen(64)
        
        self.minus = minus
        self.only = only
        
        
    def image_cb(self, msg):
        cv_image = self.draw_on(cv_image, self.overlays)
        
        cv2.imshow('image',cv_image)
        cv2.waitKey(1)
    
    def draw_on(self, img, overlays=[]):
        for i in overlays:
            img = i.draw_on(img)
        return img
        
    def overlay_cb(self, overlays =[]):
        
        if self.minus!=[]:
            for i in overlays:
                if (i.object.name in self.minus):
                    overlays.remove(i)
        
        elif self.only !=[]:
            for i in overlays:
                if (i.object.name not in self.only):
                    overlays.remove(i)
        overlays.sort(key = lambda x: x.object.name)
        
        if self.disco == True:
            self.colors = []
            self.colors = self.color_gen(len(overlays))
        i =0
        for j in overlays:
            j.color = self.colors[i]
            i+=1
            if i>=len(self.colors):
                i=0
        self.overlays = overlays
        
        
    def color_gen(self, n):
        c = []
        for i in range(n):
            c.append((randint(0,255),randint(0,255),randint(0,255)))
        return c

if __name__ == '__main__':
    rospy.init_node('visualization', anonymous=False)
    #arg parsing
    
    kwargs = {}
    #needs to be updated to work with rosparams!
    '''
    if ast.literal_eval(rospy.get_param("--disco_")) or ast.literal_eval(rospy.get_param("-d_")):
        kwargs.update({'disco':True})
        
    if ast.literal_eval(rospy.get_arg("--minus")) != []:
        kwargs.update({'minus':args.minus})
    if ast.literal_eval(rospy.get_arg("-m")) != []:
        kwargs.update({'minus':args.minus})
    
    
    if ast.literal_eval(rospy.get_arg("--only"))!=[]:
        kwargs.update({'only':args.only})
    if ast.literal_eval(rospy.get_arg("-o"))!=[]:
        kwargs.update({'only':args.only})
    '''
    visualization = visualization(**kwargs)
    rospy.spin()
    cv2.destroyAllWindows()
