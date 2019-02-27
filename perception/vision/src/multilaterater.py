#!/usr/bin/env python
import rospy

from iamge_geometry import PinholeCameraModel




class multialaterater:
    def __init__(self):
        
        rospy.Subscriber('persistent_objects_in_image', ObjectsInImage, self.objects_in_image_cb)
        
        
        
        
    def objects_in_image_cb(self):
        
        
        
        
        
        
        


if __name__ == "__main__":
    rospy.init_node("multilaterater", anonymous = False)
    mulitlaterater = multilaterator()
    rospy.spin()