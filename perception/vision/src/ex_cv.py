#!/usr/bin/env python


import rospy
from mil_msgs.msg import ObjectsInImage
from mil_msgs.msg import ObjectInImage
from mil_msgs.msg import Point2D
from sensor_msgs.msg import Image
import cv2
from math import cos, sin, pi

class ex_cv:
    def __init__(self):
        rospy.Subscriber('/camera/front/right/image_raw', Image,self.image_cb)
        self.pub=rospy.Publisher('VISION',ObjectsInImage,queue_size=1)
        self.objects_in_image = ObjectsInImage()
        
        
    def image_cb(self, msg):
        #print('ex_cv:   got image')
        self.objects_in_image.header = msg.header
        box = ObjectInImage()
        
        box.name = "box"
        
        box.points = [None]*2
        
        box.points[0]= Point2D()
        box.points[0].x = msg.width/3
        box.points[0].y = msg.height/3
        
        box.points[1]= Point2D()
        box.points[1].x = (msg.width*2)/3
        box.points[1].y = (msg.height*2)/3
        
        
        spot = ObjectInImage()
        
        spot.name = "spot"
        
        spot.points = [None]*1
        
        spot.points[0]= Point2D()
        spot.points[0].x = (msg.width*4)/5
        spot.points[0].y = (msg.height*4)/5
        
        
        strange = ObjectInImage()
        
        strange.name = "strange"
        
        strange.points = [None]*9
        
        center = [msg.width/2,msg.height/2]
        
        theta=0
        i=0
        r = 100
        while i<len(strange.points):
            strange.points[i]=Point2D()
            strange.points[i].x = (100* cos(theta))+center[0]
            strange.points[i].y = (100* sin(theta))+center[1]
            
            theta = theta+((pi*2)/len(strange.points))
            i = i+1
        
        self.objects_in_image.objects = [box, spot, strange]
        self.pub.publish(self.objects_in_image)

if __name__ == '__main__':
    rospy.init_node('ex_cv', anonymous=True)
    ex_cv = ex_cv()
    rospy.spin()



