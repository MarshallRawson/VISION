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

class visualization:
    def __init__(self):
        #Subscribers
        #TODO: utilize image sub tools so dont need hard coded image topic name
        rospy.Subscriber('/camera/front/right/image_raw', Image,self.image_cb)
        
        rospy.Subscriber('VISION', ObjectsInImage,self.objects_in_image_cb)
        
        self.objects_in_image = ObjectsInImage()
        self.features = []
        self.bridge = CvBridge()
        
        
    def image_cb(self, msg):
        #print('visualization:   got image')
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        self.features = self.cull_features(msg, self.features)
        
        cv_image = self.draw_on(cv_image, self.features)
        
        cv2.imshow('image',cv_image)
        cv2.waitKey(1)
    
    def draw_on(self, img, features=[]):
        for i in features:
            img = i.draw_on(img)
        return img
    
    def cull_features(self,msg, all_features=[]):
        #TODO: something to accomidate nodes which may publish less than once per frame
        #TODO: give each feature in curr features unique and vibrant colors
        curr_features = []
        for i in all_features:
            if i.header.stamp == msg.header.stamp:
                curr_features.append(i)
        
        colors = self.color_gen(len(curr_features))
        
        i =0
        for j in curr_features:
            j.color = colors[i]
            i+=1
        
        return curr_features
    
    def color_gen(self, n):
        #TODO: needs to make colors more vibrant, right now they are all kinda grey
        c = []
        i=0
        while i<=n-1:
            c.append((((i*1.0)/(n-1))*((256**3)-1)))
            i=i+1
        d = []
        for i in c:
            
            i2 = int((i)/256**2)
            i-= i2*(256**2)
            i1 = int((i)/256**1)
            i-= i1*(256**1)
            i0 = int((i)/256**0)
            
            d.append((i2,i1,i0))
            
        return d
    
    def objects_in_image_cb(self, objects_in_image):
        #print('visualization:   got objects')
        for i in objects_in_image.objects:
            self.features.append(Feature(header = objects_in_image.header,object_in_image = i))


if __name__ == '__main__':
    rospy.init_node('visualization', anonymous=False)
    visualization = visualization()
    rospy.spin()
    cv2.destroyAllWindows()
