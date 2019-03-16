#!/usr/bin/env python
import rospy

from image_geometry import PinholeCameraModel
from mil_vision_tools import MultilateratedTracker
from mil_msgs.msg import ObjectsInImage, ObjectInImage
from mil_ros_tools import Image_Subscriber
from sensor_msgs.msg import Image

import ast

class multilaterator:
    def __init__(self):
        
        rospy.Subscriber("persistent_objects_in_image", ObjectsInImage, self.objects_in_image_cb)
        
        rospy.Subscriber("/camera/front/left/image_rect_color", Image, self.image_cb)
        
        
        self.header = rospy.Header()
        
        
        self.camera = "/camera/front/left/image_rect_color"
        
        self.image_sub = Image_Subscriber(self.camera, self.image_cb)
        self.camera_info = self.image_sub.wait_for_camera_info()
        
        
    def image_cb(self, msg):
        self.stamp = msg.Header()
        return
        
    def objects_in_image_cb(self, objects_in_image):
	
        if self.header != objects_in_image.header:
            rospy.logerr("Image publishing and persistent_objects_in_image are out of sync.")
            return
        
        
        
        
        
        
        self.tracker.add_observation(
        
        
        
        return
        

if __name__ == '__main__':
    rospy.init_node('multilaterator', anonymous = False)
    mulitlaterator = multilaterator()
    rospy.spin()
