#!/usr/bin/env python

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import cv2


def video_publisher():
    pub = rospy.Publisher('/image', Image, queue_size=10)
    video_path = rospy.get_param('video_path')
    loop = rospy.get_param('loop')
    scale = rospy.get_param('scale')
    fps = rospy.get_param('fps')
    while not rospy.is_shutdown():
        cap = cv2.VideoCapture(video_path)
        if fps is None:
            fps = cap.get(cv2.CAP_PROP_FPS)
        rate = rospy.Rate(fps)

        success, image = cap.read()
        while success and not rospy.is_shutdown():
            width = int(image.shape[1] * scale)
            height = int(image.shape[0] * scale)
            image = cv2.resize(image, (width, height))
            message = CvBridge().cv2_to_imgmsg(image)
            pub.publish(message)
            rate.sleep()
            success, image = cap.read()

        if not loop:
            break


if __name__ == '__main__':
    rospy.init_node('video_publisher')
    try:
        video_publisher()
    except rospy.ROSInterruptException:
        pass
