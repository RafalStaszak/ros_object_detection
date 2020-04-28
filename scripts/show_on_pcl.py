#!/usr/bin/env python

import rospy
from ros_object_detection.msg import BoundingBoxes, BoundingBox
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from sensor_msgs import point_cloud2


class Callbacks:
    def __init__(self):
        self.pcl = None
        self.boxes = None
        self.new_pcl = False
        self.new_boxes = False

        rospy.Subscriber("/point_cloud", PointCloud2, self.pcl_callback)
        rospy.Subscriber("/bounding_box", BoundingBoxes, self.box_callback)
        self.pcl_pub = rospy.Publisher('/point_cloud_detection', PointCloud2, queue_size=5)

    def pcl_callback(self, data):
        self.pcl = data
        self.new_pcl = True

    def box_callback(self, data):
        self.boxes = data
        self.new_boxes = True

    def process(self):
        if self.new_pcl and self.new_boxes:
            self.new_pcl = False
            self.new_boxes = False

            data = point_cloud2.read_points(self.pcl, skip_nans=True, field_names=['x', 'y', 'z', 'rgba'])

            points = list()

            for point in data:
                x, y, z, rgba = point
                for box in self.boxes.data:
                    if box.xmin <= x/2+0.5 <= box.xmax and box.ymin <= -y/2+0.5 <= box.ymax:
                        z = 0.5
                        break
                points.append([x, y, z, rgba])

            fields = [PointField('x', 0, PointField.FLOAT32, 1),
                      PointField('y', 4, PointField.FLOAT32, 1),
                      PointField('z', 8, PointField.FLOAT32, 1),
                      # PointField('rgb', 12, PointField.UINT32, 1),
                      PointField('rgba', 12, PointField.UINT32, 1),
                      ]

            header = Header()
            header.frame_id = "map"
            pc2 = point_cloud2.create_cloud(header, fields, points)
            self.pcl_pub.publish(pc2)


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
