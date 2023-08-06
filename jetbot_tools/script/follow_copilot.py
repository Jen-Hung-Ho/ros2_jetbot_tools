#!/usr/bin/env python3

# https://github.com/opencv/opencv/issues/14884
# Importing scikit-learn related library first before importing any other libraries like opencv
from sklearn.cluster import KMeans

import cv2
import rclpy
import numpy as np
import threading
import os

from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile
from rcl_interfaces.msg import ParameterType, SetParametersResult
from ros2param.api import call_get_parameters
from vision_msgs.msg import Detection2DArray
from vision_msgs.msg import ObjectHypothesisWithPose
from vision_msgs.msg import BoundingBox2D
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError # Package to convert between ROS and OpenCV Images

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from math import radians, copysign, pi, degrees
import PyKDL

from ..include.lidar_utilities import LidarTools


# logging format
# os.environ['RCUTILS_LOG_TIME_EXPERIMENTAL'] = '1'
# os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '[{severity}] [{time:.2f}] [{name}]: {message}'
# os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '[{severity}] [{name}]: {message}'

class FollowDetectCopilot(Node):

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
        super().__init__('follow_copilot')

        # Detect parameters
        self.cmd_vel_topic = self.declare_parameter('cmd_vel_topic', '/cmd_vel').get_parameter_value().string_value
        self.node_name = self.declare_parameter('node_name', '/detectnet/detectnet').get_parameter_value().string_value
        self.class_labels = self.declare_parameter('class_labels', 'class_labels_10909380423933757363').get_parameter_value().string_value
        self.overlay_topic = self.declare_parameter('overlay_topic','/detectnet/overlay').get_parameter_value().string_value
        self.detect_topic = self.declare_parameter('detect_toipc', '/detectnet/detections').get_parameter_value().string_value
        self.speed_gain = self.declare_parameter('speed', 0.1).get_parameter_value().double_value
        self.speed_up = self.declare_parameter('speed_up', 0.15).get_parameter_value().double_value
        self.turn_gain = self.declare_parameter('turn', 0.6).get_parameter_value().double_value
        self.stop_distance = self.declare_parameter('stop_distance', 0.50).get_parameter_value().double_value
        self.FoV = self.declare_parameter('fov', 90.0).get_parameter_value().double_value
        self.score = self.declare_parameter('score', 0.7).get_parameter_value().double_value
        self.stop_count = self.declare_parameter('stop_count', 10).get_parameter_value().integer_value
        self.follow_detect = self.declare_parameter('follow_detect', True).get_parameter_value().bool_value
        self.tracking_objects = self.declare_parameter('tracking_objects', ['person']).get_parameter_value().string_array_value

        # Lidar parameters
        self.laser_topic = self.declare_parameter('laser_topic', '/scan').get_parameter_value().string_value
        self.Angle = self.declare_parameter('angle', 30).get_parameter_value().integer_value
        self.fine_tune = self.declare_parameter('fine_tune', True).get_parameter_value().bool_value
        self.linear = self.declare_parameter('linear', 0.1).get_parameter_value().double_value
        self.angular = self.declare_parameter('angular', 0.2).get_parameter_value().double_value
        self.QosReliability = self.declare_parameter('qos_reliability', False).get_parameter_value().bool_value


        # Declare and acuqire 'target_frame' parameter
        self.base_frame = self.declare_parameter(
            'base_frame', 'base_footprint').get_parameter_value().string_value
        
        self.odom_frame = self.declare_parameter(
            'odom_frame', 'odom').get_parameter_value().string_value


        # Display settings informatio
        self.get_logger().info('overlay topic: {}'.format(self.overlay_topic))
        self.get_logger().info('detect_topic : {}'.format(self.detect_topic))
        self.get_logger().info('node name    : {}'.format(self.node_name))
        self.get_logger().info('class labels : {}'.format(self.class_labels))
        self.get_logger().info('speed_gain   : {}'.format(self.speed_gain))
        self.get_logger().info('speed_up     : {}'.format(self.speed_up))
        self.get_logger().info('trun_gain    : {}'.format(self.turn_gain))
        self.get_logger().info('stop_distance: {}'.format(self.stop_distance))
        self.get_logger().info('FoV          : {}'.format(self.FoV))
        self.get_logger().info('score        : {}'.format(self.score))
        self.get_logger().info('stop_count   : {}'.format(self.stop_count))
        self.get_logger().info('follow_detect: {}'.format(self.follow_detect))
        self.get_logger().info('tracking_objects:{}'.format(self.tracking_objects))

        self.get_logger().info('cmd_vel_topic: {}'.format(self.cmd_vel_topic))
        self.get_logger().info('laser_topic: {}'.format(self.laser_topic))
        self.get_logger().info('angle    : {}'.format(self.Angle))
        self.get_logger().info('fine_tune: {}'.format(self.fine_tune))
        self.get_logger().info('linear   : {}'.format(self.linear))
        self.get_logger().info('angular  : {}'.format(self.angular))
        self.get_logger().info('base_frame:{}'.format(self.base_frame))
        self.get_logger().info('odom_frame:{}'.format(self.odom_frame))
        self.get_logger().info('QOS reliability:{}'.format(self.QosReliability))


        self.mutex = threading.Lock()
        self.class_label_names = []
        # self.tracking_objects = ['person', 'toothbrush']
        self.stop_steer = True
        self.twist = Twist()
        self.last_detection = 0

        self.sonar_samples = []
        self.lidarlib = LidarTools(self.get_logger())
        self.lidar_ready = False

        # Add parameters callback 
        self.add_on_set_parameters_callback(self.parameter_callback)

        qos_profile = QoSProfile(depth=10)
        if self.QosReliability:
            qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        else:
            qos_profile.reliability = QoSReliabilityPolicy.RELIABLE
        qos_profile.durability = QoSDurabilityPolicy.VOLATILE

        # Initialize the tf2 listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

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
        
        self.subscription = self.create_subscription(
            LaserScan, 
            self.laser_topic,
            self.laser_callback, 
            qos_profile)

        self.pub_twist = self.create_publisher(
            Twist, self.cmd_vel_topic, 10)

        self.create_timer(0.2, self.timer_callback)
    
        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()

    #
    # KMeans version laser subscription callback
    #
    def laser_callback(self, msg):
        self.get_logger().debug('Laser callback ()')

        # Collect lidar sample at 30 degrees
        # self.lidarlib.collect_lidar_data(self.Angle, msg)

        # Collect lidar raw data 
        self.lidarlib.collect_lidar_raw_data(msg)
        self.lidar_ready = True
        

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
            self.rows = image.shape[0]
            self.cols = image.shape[1]
            self.center_x = int(self.cols/2.0)
            self.center_y = int(self.rows/2.0)

        # self.get_logger().info('video :col-row[{},{}], center:[{}, {}]'.format(self.cols, self.rows, self.center_x, self.center_y))

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
            probe_distance = 0.0
            probe_angle = 0.0
            # Use bbox -- > area
            area = detection.bbox.size_x * detection.bbox.size_y
            for result in detection.results:
                # id: "\x02"  
                id = ord(result.id)
                score = result.score
                label = self.GetClassDesc(id)
                # self.get_logger().info('Result:[{}] id:[{}]=[{}] score:[{}]'.format(i, id, label, score))
                if label in self.tracking_objects and score > self.score:
                    (angle, distance) = self.fusion_object_angle_lidar_distance(detection)
                    if distance > self.stop_distance:
                        # Update tracking target whatever is closer 
                        if (follow == False) or (distance < probe_distance):
                            probe_angle = angle
                            probe_distance = distance
                            follow = True
                            self.get_logger().info('Fusion target:[{}]- [a:d]:{:.2f}:{:.2f}'.format(i, probe_angle, probe_distance))
                    else:
                        self.get_logger().info('Fusion:[{}] id:[{}]=[{}] score:[{:.2f}] distance:[{:.2f}]'.format(i, id, label, score, distance))

                i += 1
            
            if self.follow_detect == True:
                if (follow == True) and (probe_distance > self.stop_distance):
                    self.last_detection = self.stop_count
                    self.detect_and_follow(detection, probe_angle, probe_distance)
                    self.get_logger().info('Follow target-[a:d]:{:.2f}:{:.2f}'.format(probe_angle, probe_distance))
                elif (follow == True) and (probe_distance <= self.stop_distance):
                    self.get_logger().info('Too close distance={}'.format(probe_distance))



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

    #
    # Fusion the angle between camera center and detect object and lidar distance
    #
    def fusion_object_angle_lidar_distance(self, detection):
        probe_angle = 0.0
        probe_distance = 0.0

        # wait for detection overlay callback is ready
        if hasattr(self, 'center_x') and self.lidar_ready:
            # Calculate the angle between the camera and the object measured from the front of the object
            with self.mutex:
                # Calculate the angle between the camera center and detec object
                delta_x = detection.bbox.center.x - self.center_x
                probe_angle = -1.0 * (delta_x / self.center_x) * (self.FoV / 2.0)
            
            probe_distance = self.lidarlib.get_distance_from_lidar_data(probe_angle, tolerance=10.0)
            
            self.get_logger().debug('Fusion: [angle::dist]: {:.2f} : {:.2f}'.format(probe_angle, probe_distance))
           
        return probe_angle, probe_distance

        
    def detect_and_follow(self, detection, probe_angle, probe_distance):
        # self.get_logger().info('detect and follow:()'.format('hoj'))
        
        # Normalize turn angle
        # turn = probe_angle / self.Fov
        turn = probe_angle / (self.FoV/2.0)
        turn = self.saturate(turn, -1.5, 1.5) * self.turn_gain

        # if probe_distance > 0.30:
        if probe_distance > self.stop_distance:
            self.twist.linear.x = self.speed_gain
            self.twist.angular.z = turn
            # speed up if detect object is far away
            if probe_distance > 1.2:
                self.twist.linear.x += self.speed_up
        else:
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0

        # Publish Twist command
        # self.get_logger().info('Twist:{}'.format(self.twist))
        self.pub_twist.publish(self.twist)


def main(args=None):
    rclpy.init(args=args)
    
    global executor
    executor = rclpy.executors.MultiThreadedExecutor()

    
    follow_copilot_node = FollowDetectCopilot()
    executor.add_node(follow_copilot_node)

    # rclpy.spin(parameter)
    executor.spin()

    rclpy.shutdown()


if __name__ == '__main__':
    main()