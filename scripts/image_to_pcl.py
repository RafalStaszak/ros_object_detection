#!/usr/bin/env python

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np
import cv2
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import struct


def image_to_pointcloud(data):
    image = CvBridge().imgmsg_to_cv2(data)
    h, w = np.shape(image)[:2]

    image = cv2.resize(image, (100, 100 * w / h))
    h, w = np.shape(image)[:2]

    points = []

    for i, row in enumerate(image):
        for j, color in enumerate(row):
            x = (float(j) / w - 0.5) * 2
            y = -(float(i) / h - 0.5) * 2
            z = 0
            b = int(color[0])
            g = int(color[1])
            r = int(color[2])
            a = 255
            rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
            pt = [x, y, z, rgb]
            points.append(pt)


    fields = [PointField('x', 0, PointField.FLOAT32, 1),
              PointField('y', 4, PointField.FLOAT32, 1),
              PointField('z', 8, PointField.FLOAT32, 1),
              # PointField('rgb', 12, PointField.UINT32, 1),
              PointField('rgba', 12, PointField.UINT32, 1),
              ]

    header = Header()
    header.frame_id = "map"
    pc2 = point_cloud2.create_cloud(header, fields, points)
    pub.publish(pc2)


pub = rospy.Publisher('/point_cloud', PointCloud2, queue_size=10)

if __name__ == '__main__':
    rospy.init_node('pcl_publisher')
    sub = rospy.Subscriber('/image', Image, image_to_pointcloud)
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
