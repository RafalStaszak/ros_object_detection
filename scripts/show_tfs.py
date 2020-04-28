#!/usr/bin/env python

# !/usr/bin/env python

import rospy
import numpy as np
from fiducial_msgs.msg import FiducialArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ros_object_detection.msg import BoundingBoxes, BoundingBox
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from sensor_msgs import point_cloud2
import tf


class Callbacks:
    def __init__(self):
        self.boxes = None

        rospy.Subscriber("/bounding_box", BoundingBoxes, self.box_callback)

        self.br = tf.TransformBroadcaster()
        self.parent_tf = rospy.get_param('parent', 'map')

    def box_callback(self, data):
        self.boxes = data
        for i, box in enumerate(self.boxes.data):
            x = ((box.xmin + box.xmax) / 2 - 0.5) * 2
            y = -((box.ymin + box.ymax) / 2 - 0.5) * 2

            self.br.sendTransform([x, y, 0], [0, 0, 0, 1],
                                  rospy.Time.now(), '{}_{}'.format(box.data, i),
                                  self.parent_tf)


if __name__ == '__main__':
    rospy.init_node('show_tfs')
    callbacks = Callbacks()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
