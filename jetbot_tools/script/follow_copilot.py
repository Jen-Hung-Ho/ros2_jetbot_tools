#!/usr/bin/env python3
#
# Copyright (c) 2025, Jen-Hung Ho 
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
from ..include.node_parameter_utility import NodeParamTools

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
        self.version = self.declare_parameter('version', 2).get_parameter_value().integer_value
        self.cmd_vel_topic = self.declare_parameter('cmd_vel_topic', '/cmd_vel').get_parameter_value().string_value
        self.node_name = self.declare_parameter('node_name', '/yolo_detection_node').get_parameter_value().string_value
        self.class_labels = self.declare_parameter('class_labels', 'class_labels').get_parameter_value().string_value
        self.overlay_topic = self.declare_parameter('overlay_topic','/yolo/overlay').get_parameter_value().string_value
        self.detect_topic = self.declare_parameter('detect_topic', '/yolo/detections').get_parameter_value().string_value
        self.overlay_scale_param = self.declare_parameter('overlay_scale_param', 'overlay_scale_percent').get_parameter_value().string_value
        self.overlay_scale_percent = self.declare_parameter('overlay_scale_percent', 100).get_parameter_value().integer_value
        self.speed_gain = self.declare_parameter('speed', 0.1).get_parameter_value().double_value
        self.speed_up = self.declare_parameter('speed_up', 0.07).get_parameter_value().double_value
        self.turn_gain = self.declare_parameter('turn', 0.6).get_parameter_value().double_value
        self.stop_distance = self.declare_parameter('stop_distance', 0.50).get_parameter_value().double_value
        self.stop_zone_buffer = self.declare_parameter('stop_zone_buffer', 0.20).get_parameter_value().double_value
        self.FoV = self.declare_parameter('fov', 90.0).get_parameter_value().double_value
        self.score = self.declare_parameter('score', 0.7).get_parameter_value().double_value
        self.stop_count = self.declare_parameter('stop_count', 10).get_parameter_value().integer_value
        self.follow_detect = self.declare_parameter('follow_detect', False).get_parameter_value().bool_value
        self.tracking_objects = self.declare_parameter('tracking_objects', ['person']).get_parameter_value().string_array_value
        self.class_label_names = self.declare_parameter('class_label_names', ['']).get_parameter_value().string_array_value

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


        # Display settings information
        self.get_logger().info('---------------------------------------------------')
        self.get_logger().info('version      : {}'.format(self.version))
        self.get_logger().info('overlay topic: {}'.format(self.overlay_topic))
        self.get_logger().info('detect_topic : {}'.format(self.detect_topic))
        self.get_logger().info('overlay_scale_param: {}'.format(self.overlay_scale_param))
        self.get_logger().info('overlay_scale: {}'.format(self.overlay_scale_percent))
        self.get_logger().info('node name    : {}'.format(self.node_name))
        self.get_logger().info('class labels : {}'.format(self.class_labels))
        self.get_logger().info('speed_gain   : {}'.format(self.speed_gain))
        self.get_logger().info('speed_up     : {}'.format(self.speed_up))
        self.get_logger().info('trun_gain    : {}'.format(self.turn_gain))
        self.get_logger().info('stop_distance: {}'.format(self.stop_distance))
        self.get_logger().info('stop_zone_buffer: {}'.format(self.stop_zone_buffer))
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
        self.get_logger().info('---------------------------------------------------')


        self.mutex = threading.Lock()
        # self.class_label_names = []
        # self.tracking_objects = ['person', 'toothbrush']
        self.stop_steer = True
        self.twist = Twist()
        self.last_detection = 0

        self.sonar_samples = []
        self.lidarlib = LidarTools(self.get_logger())
        self.lidar_ready = False

        # Add parameters callback 
        self.add_on_set_parameters_callback(self.parameter_callback)

        # self.init_ros_nodes()
        self.node_param_util = NodeParamTools(self, executor)

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

        if (self.version == 2):
            self.get_logger().info('---------------------------------------------------')
            # V2 - YOLO detection
            self.get_logger().info('FollowDetectCopilot: YOLO detection version 2')

            # In node start up phase the check node exist result need retry severy several times
            # therefore can't use try_get_node_parameter, need direct call into the parameter service
            value = self.node_param_util.get_node_parameters(self.node_name, self.overlay_scale_param)
            if value is not None:
                self.get_logger().info('overlay_scale_percent: {}'.format(value.integer_value))
                self.overlay_scale_percent = value.integer_value
            else:
                self.overlay_scale_percent = 100
            self.get_logger().info('---------------------------------------------------')



    #
    # Remove nodes for get/set parameter service call
    #
    def cleanup(self):
        self.node_param_util.cleanup()
        pass

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


    #
    # Retrieve detection class description from list index
    #
    def GetClassDesc(self, index):
        if index >= len(self.class_label_names):
            self.get_logger().info('invalid class {}'.format(index))
            return "Invalid"
        else:
            return self.class_label_names[index]


    #
    # Retrieve image center point from /detectnet/detectnet node overlay output
    #
    def overlay_callback(self, data):
        # Convert ROS Image message to OpenCV image
        image = self.br.imgmsg_to_cv2(data)
        # Lock the code 
        # The mutex is automatically release when the with bolock is exited
        with self.mutex:
            # self.rows = image.shape[0]
            # self.cols = image.shape[1]
            # self.center_x = int(self.cols/2.0)
            # self.center_y = int(self.rows/2.0)
            # The sender scales the image by (scale/100.0), so reverse it here
            scale = self.overlay_scale_percent / 100.0
            # Recover the original sender-side width and height
            orig_width = int(image.shape[1] / scale)
            orig_height = int(image.shape[0] / scale)
            # Calculate the original center x, y
            self.center_x = int(orig_width / 2)
            self.center_y = int(orig_height / 2)

        # self.get_logger().info('video :col-row[{},{}], center:[{}, {}]'.format(self.cols, self.rows, self.center_x, self.center_y))


    #
    # Watchdog timer
    #
    def timer_callback(self):
        self.last_detection -= 1
        stop = 0
        debounce = 1
        if self.last_detection <= stop and self.last_detection >= (stop-debounce):
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            self.pub_twist.publish(self.twist)
            self.get_logger().info('Stop robot  ---------------')


    #
    # detectnet node detections subscription call_back
    #
    def detection_callback(self, msg):
        # self.get_logger().info('detection {}'.format(msg.detections))
        
        # get class label if label list is empty
        # if len(self.class_label_names) == 0:
        if not self.class_label_names or self.class_label_names == ['']:
            self.get_class_label_parameters()

        if not self.follow_detect:
            return

        follow = False
        i = 0
        probe_distance = 0.0
        probe_angle = 0.0
        probe_label = "Unknown"
        probe_score = 0.0

        # http://docs.ros.org/en/api/vision_msgs/html/msg/Detection2D.html
        for detection in msg.detections:

            # TODO: V1 - detectnet detection result 
            if self.version == 2:
                # V2 - YOLO detection result
                track_id, object_distance, class_id, label, score = self.extract_detection_info(detection)
            else:
                # V1 - detectnet detection result
                track_id, object_distance,_class_id, label, score = self.extract_detectnet_detection_info(detection)



            if label in self.tracking_objects and score > self.score:
                # V1 - detectnet detection result doent's have distance information -need lidar fusion
                #(angle, distance) = self.fusion_object_angle_lidar_distance(detection)
                if self.version == 2:
                    # V2 - YOLO detection result already has distance information
                    angle = self.get_object_angle(detection)
                else:
                    # V1 - detectnet detection result doent's have distance information -need lidar fusion
                    (angle, distance) = self.fusion_object_angle_lidar_distance(detection)
                    object_distance = distance

                if object_distance >= self.stop_distance:
                    # Update tracking target whatever is closer
                    if (follow == False) or (object_distance < probe_distance):
                        probe_label = label
                        probe_angle = angle
                        probe_distance = object_distance
                        probe_score = score
                        follow = True
                        self.get_logger().debug('Fusion target [{}]:{} c:{}- [angle:distance]:{:.2f}:{:.2f}'.format(i, label, probe_score,probe_angle, probe_distance))
                else:
                    self.get_logger().info('Stop zone: Fusion:[{}] id:[{}]=[{}] score:[{:.2f}] distance:[{:.2f}]'.format(i, id, label, score, object_distance))

            # increment the index
            i += 1
            # end if label in tracking_objects and score > self.score


        if (follow == True):
            self.last_detection = self.stop_count
            self.detect_and_follow(detection, probe_angle, probe_distance)
            self.get_logger().info('Follow target {} c:{:.2f} -[a:d]:{:.2f}:{:.2f}'.format(probe_label, probe_score, probe_angle, probe_distance))
        # elif (follow == True) and (probe_distance <= self.stop_distance):
        #    self.get_logger().info('Too close distance={}'.format(probe_distance))


    #
    # Extract V1 detectnet detection information from a Detection2D message
    #
    def extract_detectnet_detection_info(self, detection):
        # http://docs.ros.org/en/api/vision_msgs/html/msg/Detection2D.html

        # default values
        track_id = -1
        object_distance = None  # Not available in detectnet, will be set later via lidar
        class_id = 0
        class_label = 'Unknown'
        score = 0.0

        # Extract class_id, label and score
        for result in detection.results:
            # self.get_logger().info('result[{}] {}'.format(i, result))
            # Get detection id/class_id , score
            if hasattr(result, 'hypothesis') and hasattr(result.hypothesis, 'class_id'):
                # ROS2 Humble
                # class_id: "\x02"  
                class_id = ord(result.hypothesis.class_id)
                score = result.hypothesis.score
            else:
                # ROS2 Foxy
                class_id = ord(result.id)
                score = result.score

            class_label = self.GetClassDesc(class_id)


        return track_id, object_distance, class_id, class_label, score


    #
    # Extract V2 YOLO detection information from a Detection2D message
    #
    def extract_detection_info(self, detection):
        # detection result from YOLO detection 
        # jetbot_vision_perception:yolo_detection_node

        # Parse detection id
        if not hasattr(detection, 'id') or not detection.id:
            detection.id = "-1,0.0"
            self.get_logger().info(f"set default Detection id :{detection.id}")

        track_id, object_distance = self.parse_detection_id(detection.id)
        if track_id is None or object_distance is None:
            self.get_logger().error(f"Failed to parse detection id: {detection.id}")

        # Default values
        class_id = None
        class_label = "Unknown"
        score = 0.0

        # Extract class_id, label, and score
        if detection.results:
            result = detection.results[0]
            if hasattr(result, 'hypothesis') and hasattr(result.hypothesis, 'class_id'):
                class_id = int(result.hypothesis.class_id)
                score = result.hypothesis.score
                class_label = self.GetClassDesc(class_id)
            else:
                self.get_logger().error('YOLO Detection result does not have hypothesis or class_id')

        return track_id, object_distance, class_id, class_label, score

    #
    # Parse a custom detection id string of the form 'trackid,object_distance,'
    # Returns (track_id: int, object_distance: float)
    #
    def parse_detection_id(self, detection_id_str):
        # --- Custom detection.id encoding for downstream use ---
        # In tracking mode and if a track_id is available, encode as "track_id,object_depth"
        # In non-tracking mode, encode as "-1,object_depth"
        # This allows clients to parse both the track ID and the estimated object depth from the id field.
        try:
            parts = detection_id_str.strip().split(',')
            if len(parts) >= 2:
                track_id = int(parts[0])
                object_distance = float(parts[1])
                return track_id, object_distance
        except Exception as e:
            # Handle parsing error
            self.get_logger().error(f"Failed to parse detection id '{detection_id_str}': {e}")
        return None, None

    #
    # Retrieve /detectnet/detectnet node detection label list
    #
    def get_class_label_parameters(self):
        self.get_logger().info('DetectCopilot --: "%s"' % 'BEGIN')

        passfail, value = self.node_param_util.try_get_node_parameters(self.node_name, self.class_labels)

        if passfail:
            self.get_logger().debug("detectnet label:{}".format(value.string_array_value))
            self.class_label_names = value.string_array_value
            self.update_class_labels_parameter()
            # person index 1
            # self.get_logger().info('class name:{}'.format(self.GetClassDesc(1)))

        self.get_logger().info('param_callback --: "%s"' % 'END')

    #
    # Update class_label_names parameter
    #
    def update_class_labels_parameter(self):
        # Update class_labels to the parameter server
        self.set_parameters([
            rclpy.parameter.Parameter(
                'class_label_names',
                rclpy.Parameter.Type.STRING_ARRAY,
                self.class_label_names
            )
        ])

    def saturate(self, value, min, max):
        if value <= min:
            return(min)
        elif value >= max:
            return(max)
        else:
            return(value)

    #
    # Calculate the angle between the camera center and the detected object
    #
    def get_object_angle(self, detection):
        probe_angle = 0.0

        # wait for detection overlay callback is ready
        if hasattr(self, 'center_x'):
            # Calculate the angle between the camera and the object measured from the front of the object
            with self.mutex:
                # Calculate the angle between the camera center and detec object
                if hasattr(detection.bbox.center, 'position'):
                    # ROS2 Humble - YOLO detection
                    delta_x = detection.bbox.center.position.x - self.center_x
                probe_angle = -1.0 * (delta_x / self.center_x) * (self.FoV / 2.0)

            self.get_logger().debug('Target angle: [{:.2f}]'.format(probe_angle))

        return probe_angle

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
                if hasattr(detection.bbox.center, 'position'):
                    # ROS2 Humble
                    delta_x = detection.bbox.center.position.x - self.center_x
                else:
                    # ROS2 Foxy
                    delta_x = detection.bbox.center.x - self.center_x
                probe_angle = -1.0 * (delta_x / self.center_x) * (self.FoV / 2.0)

            # Get the distance from lidar data - 
            # the probe_angle is the angle between the camera center and the detected object
            # the lidar is 180 degree offset from the camera
            probe_distance = self.lidarlib.get_distance_from_lidar_data(probe_angle+180.0, tolerance=10.0)

            self.get_logger().debug('Fusion: [angle::dist]: {:.2f} : {:.2f}'.format(probe_angle, probe_distance))
           
        return probe_angle, probe_distance


    #
    # Caculate rotate ange by measure between camera Field of View (FOV) and detection image postion
    # Post Twist command to move robot follow the detection object
    #
    def detect_and_follow(self, detection, probe_angle, probe_distance):
        # self.get_logger().info('detect and follow:()'.format('hoj'))
        
        # Normalize turn angle
        # turn = probe_angle / self.Fov
        turn = probe_angle / (self.FoV/2.0)
        turn = self.saturate(turn, -1.5, 1.5) * self.turn_gain


        # 0.50 + 0.40 = 0.90 forward
        # 0.90 -- 0.70 stop zone
        # 0.50 + 0.20 = 0.70 reverse
        forward_zone_buffer = self.stop_zone_buffer * 2
        reverse_zone_buffer = self.stop_zone_buffer
        forward_zone_threshold = self.stop_distance + forward_zone_buffer
        reverse_zone_threshold = self.stop_distance + reverse_zone_buffer

        if probe_distance > forward_zone_threshold:
            # forward > 1.1 M
            self.twist.linear.x = self.speed_gain
            self.twist.angular.z = turn
            self.get_logger().debug('Forward: probe_distance={:.2f} --> stop_distance={:.2f}'.format(probe_distance, self.stop_distance)) 
            if probe_distance > 1.2:
                self.twist.linear.x += self.speed_up
        elif probe_distance < reverse_zone_threshold:
            # backward < 0.90
            self.twist.linear.x = self.speed_gain * -1.0
            self.twist.angular.z = turn
            self.get_logger().debug('Backward: probe_distance={:.2f} --> stop_distance={:.2f}'.format(probe_distance, self.stop_distance))
        else:
            # stop  between 0.90 and 1.1 M  or less than stop_distance
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

    try:
        # rclpy.spin(parameter)
        executor.spin()
    except KeyboardInterrupt:
        print('\ncontrol-c: follow_copilot_node_node shutting down')
    finally:
        # Destroy the node explictly - don't depend on garbage collector
        follow_copilot_node.cleanup()
        follow_copilot_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()