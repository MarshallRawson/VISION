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
        
        self.tracker = CentroidObjectsTracker(max_distance=20.0)
        
    def image_cb(self, msg):
        
        self.features = self.update_features(msg, self.features)
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        
        cv_image = self.draw_on(cv_image, self.features)
        
        cv2.imshow('image',cv_image)
        cv2.waitKey(1)
    
    def draw_on(self, img, features=[]):
        for i in features:
            img = i.draw_on(img)
        return img
    
    def update_features(self,msg, all_features=[]):
        
        persistent = self.tracker.get_persistent_objects(min_observations=8, min_age=rospy.Duration(0))
        
        curr_features = []
        
        for i in persistent:
            curr_features.append(i.data)
        
        #print"persistent: ", len(persistent)
        #print"curr_features: ", len(curr_features)
        
        
        if self.minus!=[]:
            for i in curr_features:
                if (i.object.name in self.minus):
                    curr_features.remove(i)
        
        elif self.only !=[]:
            for i in curr_features:
                if (i.object.name not in self.only):
                    curr_features.remove(i)
        
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
        self.tracker.clear_expired(now=objects_in_image.header.stamp)
        
        for i in objects_in_image.objects:
            f = Feature(header = objects_in_image.header,object_in_image = i)
            obj = self.tracker.add_observation(objects_in_image.header.stamp, np.array(f.centroid()), data=f)


if __name__ == '__main__':
    rospy.init_node('visualization', anonymous=False)
    parser = argparse.ArgumentParser()
    parser.add_argument('-d', '--disco')
    parser.add_argument('-m','--minus')
    parser.add_argument('-o','--only')
    args = parser.parse_args()
    kwargs = {}
    if args.disco=='True':
        kwargs.update({'disco':True})
    if args.minus!=None:
        kwargs.update({'minus':args.minus})
    if args.only!=None:
        kwargs.update({'only':args.only})
    
    
    visualization = visualization(**kwargs)
    rospy.spin()
    cv2.destroyAllWindows()
