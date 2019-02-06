#!/usr/bin/env python

import rospy
from mil_msgs.msg import ObjectsInImage
from mil_msgs.msg import ObjectInImage
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from numpy import uint8
from feature import Feature
from random import randint
import argparse

class visualization:
    def __init__(self, disco = False, minus = [], only = [],**kwargs):
        
        assert not (only != [] and minus !=[])
        
        #Subscribers
        #TODO: utilize image sub tools so dont need hard coded image topic name
        rospy.Subscriber('/camera/front/right/image_raw', Image,self.image_cb)
        
        rospy.Subscriber('VISION', ObjectsInImage,self.objects_in_image_cb)
        
        self.objects_in_image = ObjectsInImage()
        self.features = []
        self.bridge = CvBridge()
        
        self.disco = disco
        
        self.colors = self.color_gen(64)
        
        self.minus = minus
        self.only = only
        
    def image_cb(self, msg):
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        self.features = self.update_features(msg, self.features)
        
        cv_image = self.draw_on(cv_image, self.features)
        
        cv2.imshow('image',cv_image)
        cv2.waitKey(1)
    
    def draw_on(self, img, features=[]):
        for i in features:
            img = i.draw_on(img)
        return img
    
    def update_features(self,msg, all_features=[]):
        #TODO: something to accomidate nodes which may publish less than once per frame
        curr_features = []
        
        if self.minus!=[]:
            for i in all_features:
                if (i.header.stamp == msg.header.stamp) and (i.object.name not in self.minus):
                    curr_features.append(i)
        elif self.only !=[]:
            for i in all_features:
                    if (i.header.stamp == msg.header.stamp) and (i.object.name in self.only):
                        curr_features.append(i)
        curr_features.sort(key = lambda x: x.object.name)
        
        if self.disco == True:
            self.colors = []
            self.colors = self.color_gen(len(curr_features))
        i =0
        for j in curr_features:
            j.color = self.colors[i]
            i+=1
            if i>=len(self.colors):
                i=0
        return curr_features
    
    def color_gen(self, n):
        c = []
        for i in range(n):
            c.append((randint(0,255),randint(0,255),randint(0,255)))
        return c
    
    def objects_in_image_cb(self, objects_in_image):
        for i in objects_in_image.objects:
            self.features.append(Feature(header = objects_in_image.header,object_in_image = i))


if __name__ == '__main__':
    rospy.init_node('visualization', anonymous=False)
    parser = argparse.ArgumentParser()
    parser.add_argument('-d', '--disco')
    parser.add_argument('-m','--minus')
    parser.add_argument('-o','--only')
    args = parser.parse_args()
    kwargs = {}
    if args.disco!=None:
        kwargs.update({'disco':args.disco})
    if args.minus!=None:
        kwargs.update({'minus':args.minus})
    if args.only!=None:
        kwargs.update({'only':args.only})
    
    
    visualization = visualization(**kwargs)
    rospy.spin()
    cv2.destroyAllWindows()
