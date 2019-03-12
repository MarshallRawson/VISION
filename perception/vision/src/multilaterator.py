#!/usr/bin/env python
import rospy

from image_geometry import PinholeCameraModel
from mil_vision_tools import TrackedObject
from mil_msgs.msg import ObjectsInImage, ObjectInImage
from mil_ros_tools import Image_Subscriber
from sensor_msgs.msg import Image

import ast

class multilaterator:
    def __init__(self):
        
        rospy.Subscriber("persistent_objects_in_image", ObjectsInImage, self.objects_in_image_cb)
        
        rospy.Subscriber("/camera/front/left/image_rect_color", Image, self.image_cb)
        
        self.young_ids = set() #ids which have only published once and are current so cannot multilaterate
        self.old_ids = set()#ids which have multilaterated at least twice and are current, ids which have not been published in the last persistent_objects_in_image will be discarded
        
        
        
        
        self.camera = "/camera/front/left/image_rect_color"
        
        self.image_sub = Image_Subscriber(self.camera, self.image_cb)
        self.camera_info = self.image_sub.wait_for_camera_info()
        
        
    def image_cb(self, msg):
        
        return
        
    def objects_in_image_cb(self, objects_in_image):
        
        current_ids = set(ast.literal_eval(i.attributes)['id'] for i in objects_in_image.objects)
        
        self.old_ids = (current_ids & self.old_ids) | (current_ids & self.young_ids)
        
        self.young_ids = current_ids - self.old_ids
        
        
        print"old_ids:"
        print self.old_ids
        
        
        
        return
        

if __name__ == '__main__':
    rospy.init_node('multilaterator', anonymous = False)
    mulitlaterator = multilaterator()
    rospy.spin()
