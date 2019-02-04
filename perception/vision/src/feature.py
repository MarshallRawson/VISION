#!/usr/bin/env python

from mil_msgs.msg import ObjectInImage
from std_msgs.msg import Header
import cv2
import numpy as np

class Feature:
    def __init__(self, header,object):
        self.header = header
        self.object_in_image = object
        self.shape = ""
        
        self.color = (0,255,0)
        self.brush = 3
        
        if len(self.object_in_image.points)==0:
            self.shape = "none"
        elif len(self.object_in_image.points)==1:
            self.shape = "point"
        elif len(self.object_in_image.points)==2:
            self.shape = "rectangle"
        elif len(self.object_in_image.points)>2:
            self.shape = "polygon"
        
    def draw_on(self, img):
        if self.shape == "none":
            img = self.draw_none(img)
        elif self.shape == "point":
            img = self.draw_point(img)
        elif self.shape == "rectangle":
            img = self.draw_rectangle(img)
        elif self.shape == "polygon":
            img = self.draw_polygon(img)
        return img
    
    def draw_none(self,img):
        
        return img
    
    def draw_point(self,img):
        
        p = (int(self.object_in_image.points[0].x),int(self.object_in_image.points[0].y))
        
        img  = cv2.circle(img, p, self.brush, self.color,-1)
        
        return img
    
    def draw_rectangle(self,img):
        p0 = (int(self.object_in_image.points[0].x),int(self.object_in_image.points[0].y))
        p1 = (int(self.object_in_image.points[1].x),int(self.object_in_image.points[1].y))
        
        img = cv2.rectangle(img, p0,p1,self.color,self.brush)
        
        return img
    
    def draw_polygon(self,img):
        p = []
        for i in self.object_in_image.points:
            p.append([int(i.x),int(i.y)])
        
        pts = np.array(p, np.int32)
        pts = pts.reshape((-1,1,2))
        img = cv2.polylines(img,[pts],True,self.color,self.brush)
        return img



