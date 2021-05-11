#!/usr/bin/env python3

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import cv2
import six
import tensorflow as tf
import numpy as np
import os
import time
from object_detection.utils import ops as utils_ops
from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as vis_util
from ros_object_detection.msg import BoundingBoxes, BoundingBox

# patch tf1 into `utils.ops`
utils_ops.tf = tf.compat.v1

# Patch the location of gfile
tf.gfile = tf.io.gfile


def version(v):
    return tuple(map(int, (v.split("."))))


if version(tf.__version__) < version('2.0.0'):
    tf.enable_eager_execution()


class ObjectDetection:

    def __init__(self, model_dir, label_map, min_score):
        self.model = self.load_model(model_dir)
        self.category_index = label_map_util.create_category_index_from_labelmap(label_map, use_display_name=True)
        self.min_score = min_score

        self.image_pub = rospy.Publisher('/image_annotated', Image, queue_size=1)
        self.box_pub = rospy.Publisher('/bounding_box', BoundingBoxes, queue_size=1)
        self.sub = rospy.Subscriber('/image', Image, self.detect)
        self.subscribe_lock = False

    def load_model(self, model_dir):
        model_dir = os.path.join(model_dir, 'saved_model')
        model = tf.compat.v2.saved_model.load(model_dir, None)
        # model = tf.saved_model.load(model_dir)
        model = model.signatures['serving_default']

        return model

    def run_inference_for_single_image(self, image):
        image = np.asarray(image)
        # The input needs to be a tensor, convert it using `tf.convert_to_tensor`.
        input_tensor = tf.convert_to_tensor(image)
        # The model expects a batch of images, so add an axis with `tf.newaxis`.
        input_tensor = input_tensor[tf.newaxis, ...]

        # Run inference
        output_dict = self.model(input_tensor)

        # All outputs are batches tensors.
        # Convert to numpy arrays, and take index [0] to remove the batch dimension.
        # We're only interested in the first num_detections.
        num_detections = int(output_dict.pop('num_detections'))
        output_dict = {key: value[0, :num_detections].numpy()
                       for key, value in output_dict.items()}
        output_dict['num_detections'] = num_detections

        # detection_classes should be ints.
        output_dict['detection_classes'] = output_dict['detection_classes'].astype(np.int64)

        # Handle models with masks:
        if 'detection_masks' in output_dict:
            # Reframe the the bbox mask to the image size.
            detection_masks_reframed = utils_ops.reframe_box_masks_to_image_masks(
                output_dict['detection_masks'], output_dict['detection_boxes'],
                image.shape[0], image.shape[1])
            detection_masks_reframed = tf.cast(detection_masks_reframed > 0.5,
                                               tf.uint8)
            output_dict['detection_masks_reframed'] = detection_masks_reframed.numpy()

        return output_dict

    def annotate_inference(self, image):
        # the array based representation of the image will be used later in order to prepare the
        # result image with boxes and labels on it.
        # Actual detection.
        output_dict = self.run_inference_for_single_image(image)
        # Visualization of the results of a detection.
        output_image = np.copy(image)
        vis_util.visualize_boxes_and_labels_on_image_array(
            output_image,
            output_dict['detection_boxes'],
            output_dict['detection_classes'],
            output_dict['detection_scores'],
            self.category_index,
            instance_masks=output_dict.get('detection_masks_reframed', None),
            use_normalized_coordinates=True,
            line_thickness=8,
            min_score_thresh=self.min_score)

        boxes, classes, scores = list(), list(), list()

        for box, cls, score in zip(output_dict['detection_boxes'], output_dict['detection_classes'],
                                   output_dict['detection_scores']):
            if score >= self.min_score:
                if cls in six.viewkeys(self.category_index):
                    class_name = self.category_index[cls]['name']
                    boxes.append(box)
                    classes.append(class_name)
                    scores.append(score)

        return output_image, boxes, classes, scores

    def detect(self, data):

        if self.subscribe_lock is False:
            self.subscribe_lock = True
            bridge = CvBridge()
            image = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
            annotated, boxes, classes, scores = self.annotate_inference(image)
            message = bridge.cv2_to_imgmsg(annotated)
            self.image_pub.publish(message)

            box_data = list()

            for box, cls, score in zip(boxes, classes, scores):
                box_msg = BoundingBox()
                box_msg.data = cls
                box_msg.confidence = score
                box_msg.xmin = box[1]
                box_msg.ymin = box[0]
                box_msg.xmax = box[3]
                box_msg.ymax = box[2]
                box_data.append(box_msg)
            self.box_pub.publish(BoundingBoxes(box_data))

            self.subscribe_lock = False
        else:
            print('Not Entered')


if __name__ == '__main__':
    rospy.init_node('object_detection')
    detection = ObjectDetection(rospy.get_param('model'), rospy.get_param('label_map'), rospy.get_param('score_thresh'))

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
