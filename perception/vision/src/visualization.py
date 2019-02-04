#!/usr/bin/env python

import rospy
from mil_msgs.msg import ObjectsInImage
from mil_msgs.msg import ObjectInImage
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError

from feature import Feature

class visualization:
    def __init__(self):
        #Subscribers
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
        #TODO something to accomidate nodes which may publish less than once per frame
        curr_features = []
        for i in all_features:
            if i.header.stamp == msg.header.stamp:
                curr_features.append(i)
        return curr_features
    
    def objects_in_image_cb(self, objects_in_image):
        #print('visualization:   got objects')
        for i in objects_in_image.objects:
            self.features.append(Feature(objects_in_image.header,i))


if __name__ == '__main__':
    rospy.init_node('visualization', anonymous=False)
    visualization = visualization()
    rospy.spin()
    cv2.destroyAllWindows()
