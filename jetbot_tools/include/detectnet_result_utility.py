#!/usr/bin/env python3
#
# Copyright (c) 2024, Jen-Hung Ho 
#
# Permission is hereby granted, free of charge, to any person obtaining a
# copy of this software and associated documentation files (the "Software"),
# to deal in the Software without restriction, including without limitation
# the rights to use, copy, modify, merge, publish, distribute, sublicense,
# and/or sell copies of the Software, and to permit persons to whom the
# Software is furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
# DEALINGS IN THE SOFTWARE.
#

import cv2
import rclpy  # Python library for ROS 2
import threading

from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray
from vision_msgs.msg import ObjectHypothesisWithPose
from vision_msgs.msg import BoundingBox2D
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError # Package to convert between ROS and OpenCV Images

# from jetbot_tools.include.node_parameter_utility import NodeParamTools

class DetectNetResult():

    def __init__(self, node, node_param_util, node_name, class_labels, FoV):

        self.node = node
        self.logger = self.node.get_logger()
        # the client node executor need to add 'get_param_node', 'set_param_node'
        self.node_param_util = node_param_util
        self.node_name = node_name
        self.class_labels = class_labels
        self.FoV = FoV  # field of view (FOV)

        self.mutex = threading.Lock()
        self.class_label_names = []
        self.get_class_label_parameters()

        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()


    #
    # Retrieve /detectnet/detectnet node detection label list
    #
    def get_class_label_parameters(self):

        passfail, value = self.node_param_util.try_get_node_parameters(self.node_name, self.class_labels)

        if passfail:
            self.logger.info("detectnet label:{}".format(value.string_array_value))
            self.class_label_names = value.string_array_value
            # person index 1
            # self.get_logger().info('class name:{}'.format(self.GetClassDesc(1)))


    #
    # Retrieve detection class description from list index
    #
    def GetClassDesc(self, index):
        if index >= len(self.class_label_names):
            self.logger.info('invalid class {}'.format(index))
            return "Invalid"
        else:
            return self.class_label_names[index]

    #
        
    # Retrieve the angle between camera center and detect object
    #
    def get_object_angle(self, detection):
        probe_angle = 0.0

        # wait for detection overlay callback is ready
        if hasattr(self, 'center_x'):
            # Calculate the angle between the camera and the object measured from the front of the object
            with self.mutex:
                # Calculate the angle between the camera center and detec object
                if hasattr(detection.bbox.center, 'position'):
                    # ROS2 Humble
                    delta_x = detection.bbox.center.position.x - self.center_x
                else:
                    # ROS2 Foxy
                    delta_x = detection.bbox.center.x - self.center_x
                probe_angle = -1.0 * (delta_x / self.center_x) * (self.FoV / 2.0)

            self.logger.debug('Detection: [angle]: {:.2f} '.format(probe_angle))
           
        return probe_angle

    #
    # Detectnet imaged overlayere with the detection results
    #
    def overlay_image(self, data):

        # Convert ROS Image message to OpenCV image
        image = self.br.imgmsg_to_cv2(data)
        # Lock the code 
        # The mutex is automatically release when the with bolock is exited
        with self.mutex:
            self.rows = image.shape[0]
            self.cols = image.shape[1]
            self.center_x = int(self.cols/2.0)
            self.center_y = int(self.rows/2.0)

        self.logger.debug('video :col-row[{},{}], center:[{}, {}]'.format(self.cols, self.rows, self.center_x, self.center_y))
        pass

    #
    # detectnet detecton result array 
    #
    def target_in_detections(self, tracking_objects, score, msg):

        # "tracking_objects" is a list of strings where each string represents an interesting tracking target. 
        # These targets are the output results from a detectnet module.

        # get class label if label list is empty
        if len(self.class_label_names) == 0:
            self.get_class_label_parameters()
            return (False, None, None, None)

        for detection in msg.detections:
            # area = detection.bbox.size_x * detection.bbox.size_y
            for result in detection.results:
                if hasattr(result, 'hypothesis') and hasattr(result.hypothesis, 'class_id'):
                    # ROS2 Humble
                    # class_id: "\x02"  
                    id = ord(result.hypothesis.class_id)
                    d_score = result.hypothesis.score
                else:
                    # ROS2 Foxy
                    # id: "\x02"  
                    id = ord(result.id)
                    d_score = result.score
                label = self.GetClassDesc(id)
                self.logger.debug('Result id:[{}]=[{}] score:[{}]'.format(id, label, d_score))
                if label in tracking_objects and d_score > score:
                    angle = self.get_object_angle(detection)
                    return (True, label, d_score, angle)
                else:
                    self.logger.debug('Detect:{}, score:{}'.format(label, d_score))

        return (False, None, None, None)