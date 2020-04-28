#!/usr/bin/env python

import rospy
from ros_object_detection.msg import BoundingBoxes, BoundingBox
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Header
from sensor_msgs import point_cloud2
import numpy as np


class Callbacks:
    def __init__(self):
        self.image = np.zeros((100, 100, 3), dtype=np.uint8)
        self.h, self.w = np.shape(self.image)[:2]
        self.car = [255, 0, 0]
        self.pedestrians = [0, 255, 0]
        self.bicycle = [0, 0, 255]
        rospy.Subscriber("/bounding_box", BoundingBoxes, self.box_callback)
        self.image_pub = rospy.Publisher('/trajectory', Image, queue_size=5)

    def box_callback(self, boxes):
        for box in boxes.data:
            x = (box.xmin + box.xmax) / 2
            y = (box.ymin + box.ymax) / 2
            color = [0, 0, 0]
            if box.data == 'car':
                color = self.car
            elif box.data == 'pedestrians':
                color = self.pedestrians
            elif box.data == 'bicycle':
                color = self.bicycle

            self.image[int(y * self.h), int(x * self.w), :] = color

    def process(self):
        self.image_pub.publish(CvBridge().cv2_to_imgmsg(self.image))


if __name__ == '__main__':
    rospy.init_node('highlight_markers')
    callbacks = Callbacks()
    try:
        rate = rospy.Rate(100)  # 100hz
        while not rospy.is_shutdown():
            callbacks.process()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
