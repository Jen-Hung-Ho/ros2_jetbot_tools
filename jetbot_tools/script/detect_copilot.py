#!/usr/bin/env python3
#
# Copyright (c) 2023, Jen-Hung Ho 
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
import rclpy
import numpy as np
import threading
import os

from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterType, SetParametersResult
from ros2param.api import call_get_parameters
from vision_msgs.msg import Detection2DArray
from vision_msgs.msg import ObjectHypothesisWithPose
from vision_msgs.msg import BoundingBox2D
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError # Package to convert between ROS and OpenCV Images

# logging format
os.environ['RCUTILS_LOG_TIME_EXPERIMENTAL'] = '1'
# os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '[{severity}] [{time:.2f}] [{name}]: {message}'
os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '[{severity}] [{name}]: {message}'

class DetectCopilot(Node):

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'follow_detect' and param.type_ == Parameter.Type.BOOL:
                self.follow_detect = param.value
                self.get_logger().info('follow_detect= {}'.format(bool(param.value)))
            elif param.name == 'stop_count' and param.type_ == Parameter.Type.INTEGER:
                self.get_logger().info('stop count= {}'.format(str(param.value)))
                self.tolerance = param.value
        return SetParametersResult(successful=True)

    def __init__(self):
        super().__init__('detect_copilot')

        self.cmd_vel_topic = self.declare_parameter('cmd_vel_topic', '/cmd_vel').get_parameter_value().string_value
        self.node_name = self.declare_parameter('node_name', '/detectnet/detectnet').get_parameter_value().string_value
        self.class_labels = self.declare_parameter('class_labels', 'class_labels_10909380423933757363').get_parameter_value().string_value
        self.overlay_topic = self.declare_parameter('overlay_topic','/detectnet/overlay').get_parameter_value().string_value
        self.detect_topic = self.declare_parameter('detect_toipc', '/detectnet/detections').get_parameter_value().string_value
        self.speed_gain = self.declare_parameter('speed', 0.1).get_parameter_value().double_value
        self.turn_gain = self.declare_parameter('turn', 0.6).get_parameter_value().double_value
        self.stop_size = self.declare_parameter('size', 28000).get_parameter_value().integer_value
        self.score = self.declare_parameter('score', 0.8).get_parameter_value().double_value
        self.stop_count = self.declare_parameter('stop_count', 7).get_parameter_value().integer_value
        self.follow_detect = self.declare_parameter('follow_detect', True).get_parameter_value().bool_value
        self.tracking_objects = self.declare_parameter('tracking_objects', ['kendama']).get_parameter_value().string_array_value

        self.get_logger().info('cmd_vel_topic: {}'.format(self.cmd_vel_topic))
        self.get_logger().info('overlay topic: {}'.format(self.overlay_topic))
        self.get_logger().info('detect_topic : {}'.format(self.detect_topic))
        self.get_logger().info('node name    : {}'.format(self.node_name))
        self.get_logger().info('class labels : {}'.format(self.class_labels))
        self.get_logger().info('seed_gain    : {}'.format(self.speed_gain))
        self.get_logger().info('trun_gain    : {}'.format(self.turn_gain))
        self.get_logger().info('stop_size    : {}'.format(self.stop_size))
        self.get_logger().info('score        : {}'.format(self.score))
        self.get_logger().info('stop_count   : {}'.format(self.stop_count))
        self.get_logger().info('follow_detect: {}'.format(self.follow_detect))
        self.get_logger().info('tracking_objects:{}'.format(self.tracking_objects))

        self.mutex = threading.Lock()
        self.class_label_names = []
        # self.tracking_objects = ['kendama', 'yoyo']
        self.stop_steer = True
        self.twist = Twist()
        self.last_detection = 0

        # Add parameters callback 
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Create the subscriber. This subscriber will receive an Image
        # from the detectnet overlay video_frames topic. The queue size is 10 messages.
        self.subscription = self.create_subscription(
            Image, 
            self.overlay_topic, 
            self.overlay_callback, 
            10)

        self.subscription = self.create_subscription(
            Detection2DArray,
            self.detect_topic,
            self.detection_callback,
            10)
        
        self.pub_twist = self.create_publisher(
            Twist, self.cmd_vel_topic, 10)

        self.create_timer(0.2, self.timer_callback)
    
        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()

    def GetClassDesc(self, index):
        if index >= len(self.class_label_names):
            self.get_logger().info('invalid class {}'.format(index))
            return "Invalid"
        else:
            return self.class_label_names[index]

    def overlay_callback(self, data):
        # Convert ROS Image message to OpenCV image
        image = self.br.imgmsg_to_cv2(data)
        # Lock the code 
        # The mutex is automatically release when the with bolock is exited
        with self.mutex:
            rows = image.shape[0]
            cols = image.shape[1]
            self.center_x = int(cols/2.0)
            self.center_y = int(rows/2.0)

        # self.get_logger().info('video :col-row[{},{}], center:[{}, {}]'.format(cols, rows, self.center_x, self.center_y))

    def timer_callback(self):
        self.last_detection -= 1
        stop = 0
        debounce = 1
        if self.last_detection <= stop and self.last_detection >= (stop-debounce):
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            self.pub_twist.publish(self.twist)
            self.get_logger().info('Stop robot  ---------------')

    def detection_callback(self, msg):
        # self.get_logger().info('detection {}'.format(detections))
        
        # get class label if label list is empty
        if len(self.class_label_names) == 0:
            self.get_class_label_parameters()


        for detection in msg.detections:
            # http://docs.ros.org/en/api/vision_msgs/html/msg/Detection2D.html
        
            follow = False
            i = 0
            # Use bbox -- > area
            area = detection.bbox.size_x * detection.bbox.size_y
            for result in detection.results:
                # id: "\x02"  
                id = ord(result.id)
                score = result.score
                label = self.GetClassDesc(id)
                # self.get_logger().info('Result:[{}] id:[{}]=[{}] score:[{}]'.format(i, id, label, score))
                if label in self.tracking_objects and score > self.score:
                    follow = True
                    self.get_logger().info('Result:[{}] id:[{}]=[{}] score:[{:.2f}] area:[{:.2f}]'.format(i, id, label, score, area))
                    break
                i += 1
            
            if self.follow_detect == True:
                if follow == True and area < self.stop_size / 1.0:
                    self.last_detection = self.stop_count
                    self.detect_and_follow(detection)
                elif follow == True and area > self.stop_size / 1.0:
                    self.get_logger().info('Too close area={}'.format(area))

            
    def get_class_label_parameters(self):
        self.get_logger().info('DetectCopilot --: "%s"' % 'BEGIN')

        node = rclpy.create_node('dummy_node')
        executor.add_node(node)
        # parameters = ['class_labels_10909380423933757363']
        parameters = [self.class_labels]
        response = call_get_parameters(node=node, 
                                #node_name='/detectnet/detectnet', 
                                node_name=self.node_name,
                                parameter_names=parameters)
        
        if len(response.values) >= 1:
            # print(response.values)
            # txtract type specific value
            pvalue = response.values[0]
            if pvalue.type == ParameterType.PARAMETER_STRING_ARRAY:
                # print(pvalue.string_array_value)
                self.get_logger().info('DetectCopilot --: "%s"' % pvalue.string_array_value)
                self.class_label_names = pvalue.string_array_value

            self.get_logger().info('class name:{}'.format(self.GetClassDesc(0)))
            self.get_logger().info('class name:{}'.format(self.GetClassDesc(1)))
            self.get_logger().info('class name:{}'.format(self.GetClassDesc(2)))
            self.get_logger().info('class name:{}'.format(self.GetClassDesc(3)))

        executor.remove_node(node)
        node.destroy_node()

        self.get_logger().info('param_callback --: "%s"' % 'END')

    
    def saturate(self, value, min, max):
        if value <= min:
            return(min)
        elif value >= max:
            return(max)
        else:
            return(value)

        
    def detect_and_follow(self, detection):
        # self.get_logger().info('detect and follow:()'.format('hoj'))

        # 1) calculate Area
        area = detection.bbox.size_x * detection.bbox.size_y
        turn = 0.0
        self.twist.linear.x = 0.0
        self.twist.angular.z = turn

        # 2) deltime rotate angle
        with self.mutex:
            delta_x = detection.bbox.center.x - self.center_x
            turn = -1.0 * (delta_x / self.center_x)
        
        # delta_x = detection.bbox.center.x - self.center_x
        # turn = -1.0 * (delta_x / self.center_x)
        turn = self.saturate(turn, -1.5, 1.5) * self.turn_gain

        s_x = detection.bbox.size_x
        s_y = detection.bbox.size_y
        x = detection.bbox.center.x
        y = detection.bbox.center.y
        self.get_logger().info('detect area:{:.1f} - turn:{:.1f}  center:[x:{:.1f} y:{:.1f}] size: [s_x:{:.1f} s_y:{:.1f}] '.format(area, turn, x, y, s_x, s_y))

        if area < (self.stop_size / 1.0) and abs(turn) > 0.02:
            self.twist.linear.x = self.speed_gain
            self.twist.angular.z = turn
        
        # 3) publishe Twist command
        # self.get_logger().info('Twist:{}'.format(self.twist))
        self.pub_twist.publish(self.twist)



def main(args=None):
    rclpy.init(args=args)
    
    global executor
    executor = rclpy.executors.MultiThreadedExecutor()

    
    detect_copilot_node = DetectCopilot()
    executor.add_node(detect_copilot_node)

    # rclpy.spin(parameter)
    executor.spin()

    rclpy.shutdown()


if __name__ == '__main__':
    main()