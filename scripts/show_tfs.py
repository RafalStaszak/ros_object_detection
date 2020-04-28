#!/usr/bin/env python

# !/usr/bin/env python

import rospy
import tf
from ros_object_detection.msg import BoundingBoxes


class Callbacks:
    def __init__(self):
        self.boxes = None

        rospy.Subscriber("/bounding_box", BoundingBoxes, self.box_callback)


    def box_callback(self, data):
        self.boxes = data
        for i, box in enumerate(self.boxes.data):
            pass

if __name__ == '__main__':
    rospy.init_node('show_tfs')
    callbacks = Callbacks()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
