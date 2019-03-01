#!/usr/bin/env python
import rospy

from iamge_geometry import PinholeCameraModel
from mil_vision_tools import TrackedObject




class multialaterator:
    def __init__(self):
        
        rospy.Subscriber('tracked_objects', TrackedObject, self.objects_in_image_cb)
        
        
        
        
    def objects_in_image_cb(self):
        
        
        
        
        
        
        


if __name__ == "__main__":
    rospy.init_node("multilaterator", anonymous = False)
    mulitlaterater = multilaterator()
    rospy.spin()