#!/usr/bin/env python


import rospy
from mil_msgs.msg import ObjectsInImage
from mil_msgs.msg import ObjectInImage
from mil_msgs.msg import Point2D
from sensor_msgs.msg import Image
import cv2

class ex_cv:
    def __init__(self):
        rospy.Subscriber('/camera/front/right/image_raw', Image,self.image_cb)
        self.pub=rospy.Publisher('VISION',ObjectsInImage,queue_size=1)
        self.objects_in_image = ObjectsInImage()
        
        
    def image_cb(self, msg):
        print('ex_cv:   got image')
        self.objects_in_image.header = msg.header
        human = ObjectInImage()
        
        human.name = "human"
        
        human.points = [None]*2
        
        human.points[0]= Point2D()
        human.points[0].x = msg.width/3
        human.points[0].y = msg.height/3
        
        human.points[1]= Point2D()
        human.points[1].x = (msg.width*2)/3
        human.points[1].y = (msg.height*2)/3
        
        self.objects_in_image.objects = [human]
        self.pub.publish(self.objects_in_image)

if __name__ == '__main__':
    rospy.init_node('ex_cv', anonymous=True)
    ex_cv = ex_cv()
    rospy.spin()



