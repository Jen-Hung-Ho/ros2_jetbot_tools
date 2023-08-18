#!/usr/bin/env python3

# https://github.com/opencv/opencv/issues/14884
# Importing scikit-learn related library first before importing any other libraries like opencv
from sklearn.cluster import KMeans

import cv2
import rclpy
import numpy as np
import threading
import time

from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.parameter import Parameter
from collections import namedtuple
from rcl_interfaces.msg import ParameterType, SetParametersResult
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from geometry_msgs.msg import Twist

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from math import radians, copysign, pi, degrees

from ..include.lidar_utilities import LidarTools
from ..include.tf2_utilities import *

# logging format
# os.environ['RCUTILS_LOG_TIME_EXPERIMENTAL'] = '1'
# os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '[{severity}] [{time:.2f}] [{name}]: {message}'
# os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '[{severity}] [{name}]: {message}'

class LaserCopilot(Node):

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'start' and param.type_ == Parameter.Type.BOOL:
                self.start = param.value
                self.get_logger().info('start= {}'.format(bool(param.value)))
            elif param.name == 'stop_distance' and param.type_ == Parameter.Type.DOUBLE:
                self.get_logger().info('stop distance= {}'.format(str(param.value)))
                self.stop_distance = param.value
                self.get_logger().info('distance: {}'.format(self.stop_distance))
        return SetParametersResult(successful=True)

    def __init__(self):
        super().__init__('laser_avoidance')

        self.cmd_vel_topic = self.declare_parameter('cmd_vel_topic', '/cmd_vel').get_parameter_value().string_value
        self.QosReliability = self.declare_parameter('qos_reliability', False).get_parameter_value().bool_value
        self.laser_topic = self.declare_parameter('laser_topic', '/scan').get_parameter_value().string_value
        self.stop_distance = self.declare_parameter('stop_distance', 0.5).get_parameter_value().double_value
        self.Angle = self.declare_parameter('angle', 30).get_parameter_value().integer_value
        self.fine_tune = self.declare_parameter('fine_tune', True).get_parameter_value().bool_value
        self.linear = self.declare_parameter('linear', 0.1).get_parameter_value().double_value
        self.angular = self.declare_parameter('angular', 0.2).get_parameter_value().double_value
        self.start = self.declare_parameter('start', False).get_parameter_value().bool_value
        self.use_namespace = self.declare_parameter('use_namespace', True).get_parameter_value().bool_value

        # Declare and acuqire 'target_frame' parameter
        self.base_frame = self.declare_parameter(
            'base_frame', 'base_footprint').get_parameter_value().string_value

        self.odom_frame = self.declare_parameter(
            'odom_frame', 'odom').get_parameter_value().string_value

        # Twist -  linear, angular
        self.get_logger().info('cmd_vel_topic : {}'.format(self.cmd_vel_topic))
        self.get_logger().info('laser_topic   : {}'.format(self.laser_topic))
        self.get_logger().info('stop distance : {}'.format(self.stop_distance))
        self.get_logger().info('angle         : {}'.format(self.Angle))
        self.get_logger().info('fine_tune     : {}'.format(self.fine_tune))
        self.get_logger().info('linear        : {}'.format(self.linear))
        self.get_logger().info('angular       : {}'.format(self.angular))
        self.get_logger().info('use_namespace : {}'.format(self.use_namespace))
        self.get_logger().info('base_frame    : {}'.format(self.base_frame))
        self.get_logger().info('odom_frame    : {}'.format(self.odom_frame))
        self.get_logger().info('QOS reliability:{}'.format(self.QosReliability))
        self.get_logger().info('start         : {}'.format(self.start))

        self.mutex = threading.Lock()
        self.warning = [0, 0, 0] #(left,middle,right)
        self.sonar_samples = []
        self.stop_steer = True
        self.twist = Twist()
        self.last_move = 0
        self.rotate = False
        self.rotate_status = namedtuple('turn', ['done', 'turn_angle', 'distance', 'delta'])
        self.turn_status = self.rotate_status(False, 90, 0.0, 5)


        self.lidarlib = LidarTools(self.get_logger())

        # Add parameters callback 
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Initialize the tf2 listener
        self.tf_buffer = Buffer()
        if not self.use_namespace:
            self.tf_listener = TransformListener(self.tf_buffer, self)
        else:
            namespace = get_tf2_namespace(self.base_frame)
            self.get_logger().info('NamesapceTransformLintenter:({})'.format(namespace))
            # Default disable broacast to /tf and /tf_static
            self.global_ns = False
            self.tf_listener = NamespaceTransformLintener(namespace, self.tf_buffer, self)

        qos_profile = QoSProfile(depth=10)
        if self.QosReliability:
            qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        else:
            qos_profile.reliability = QoSReliabilityPolicy.RELIABLE
        qos_profile.durability = QoSDurabilityPolicy.VOLATILE

        # Create the subscriber. This subscriber will receive lidar message
        self.subscription = self.create_subscription(
            LaserScan, 
            self.laser_topic,
            self.laser_callback,
            qos_profile)

        self.pub_twist = self.create_publisher(
            Twist, self.cmd_vel_topic, 20)
        
        self.create_timer(0.1, self.timer_callback)

        # Need to implement multi thread node 
        self.thread = threading.Thread(target=self.rotate_callback, daemon=True)
        self.thread.start()


    #
    # KMeans version laser subscription callback
    #
    def laser_callback(self, msg):
        # Collect lidar sample at 30 degrees
        self.lidarlib.collect_lidar_data(self.Angle, msg)
        self.lidarlib.collect_lidar_raw_data(msg)

    #
    #   timer: move the robot base on the lidar data feedback
    #
    def timer_callback(self):
    
        # Do nothing if start is not trigger
        if self.start != True:
            if self.last_move < 2:
                # Stop the robot
                self.pub_twist.publish(Twist())
                self.get_logger().debug('Stop the robot')
                self.rotate = False
                # debunce
                self.last_move += 1

            return


        self.last_move = 0

        if self.rotate != True:
                        
            lidar_samples = self.lidarlib.get_lidar_samples()

            # Find the front obstacle distance
            (distance, angle, index) = self.lidarlib.front_lidar_distance(lidar_samples)
            
            if distance < self.stop_distance + 1.2:
                # str_lidar_info = '%.2f:%.2f' %(distance, angle)
                self.get_logger().info('Front:[d:a]:{:.2f}:{:.2f}'.format(distance, angle))
                self.get_logger().debug('lidar_data:{}'.format(lidar_samples))

            # fine_tune = False
            fine_tune_closeup_area = 1
            fine_tune_angle = 0.0
            fine_tune_speed = 0.0
            a_distance = [0]*3
            a_angle = [0]*3

            # Detect obstacle in close up area
            if self.fine_tune and fine_tune_closeup_area == 1:
                a_distance[1] = distance
                a_angle[1] = angle
                (a_distance[0], a_angle[0]) = lidar_samples[index-1]
                (a_distance[2], a_angle[2]) = lidar_samples[index+1]


                if ((a_distance[0] <= a_distance[2]) and (a_distance[0] <= (self.stop_distance + 0.05))):
                    # right
                    fine_tune_angle = 0.15
                    fine_tune_closeup_area = 2
                    str_lidar_info = 'Closeup area -> RIGHT -- [d:a]:%.2f:%.2f' %(a_distance[0], a_angle[0])
                elif ((a_distance[2] <= a_distance[0]) and (a_distance[2] <= (self.stop_distance + 0.05))):
                    # left
                    fine_tune_angle = -0.15
                    fine_tune_closeup_area = 2
                    str_lidar_info = 'Closeup area -> LEFT -- [d:a]:%.2f:%.2f' %(a_distance[2], a_angle[2])


                if fine_tune_closeup_area == 2:
                    self.get_logger().info('{}'.format(str_lidar_info))
                #   self.lidarlib.max_lidar_distance(lidar_samples, -70.0, 70.0, debug=True)


            # Detect obstacle in open area
            if self.fine_tune:
                (probe_distance, probe_angle) = self.lidarlib.max_lidar_distance(lidar_samples, -70.0, 70.0, debug=False)
                str_lidar_info = '%.2f:%.2f' %(probe_distance, probe_angle)
                self.get_logger().debug('Fine tune: [d:a]:{}'.format(str_lidar_info))
                if probe_distance > (distance + 0.1):
                    if fine_tune_closeup_area == 1:
                        # open area speed up
                        fine_tune_speed = 0.05
                    # turn base on probe angle
                    if probe_angle < angle:
                        fine_tune_angle += -0.15
                    else: 
                        fine_tune_angle += 0.15
    

            # debug output
            with np.printoptions(precision=3, suppress=True):
                self.get_logger().debug('numpy sample block front:[d,a]:{}:{} array:{}'.format(distance, angle, lidar_samples))

            # Front find obstacle - turn the robot
            if distance < self.stop_distance:
                # First stop the robot
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
                self.pub_twist.publish(self.twist)
                self.get_logger().info('Detect obstacle Stop robot (dump info)  ---------------')
                # self.lidarlib.max_lidar_distance(lidar_samples, -70.0, 70.0, debug=True)
                self.lidarlib.max_lidar_distance(lidar_samples, 110.0, 250.0, debug=True)

                # -90-90 Find the front possible direction to rotate
                # (distance, angle) = self.lidarlib.max_lidar_distance(lidar_samples, -90.0, 90.0)
                (distance, angle) = self.lidarlib.max_lidar_distance(lidar_samples, 90.0, 270.0)
                if distance < self.stop_distance:
                    self.get_logger().info('Look around 360 rotate degree  ---------------')
                    # Find all the possible direction to rotate
                    # (distance, angle) = self.lidarlib.max_lidar_distance(lidar_samples, -180.0, 180.0)
                    (distance, angle) = self.lidarlib.max_lidar_distance(lidar_samples, 0.0, 360.0)

                if distance > self.stop_distance:
                    self.turn_status = self.turn_status._replace(done=False)
                    self.turn_status = self.turn_status._replace(turn_angle=angle)
                    self.turn_status = self.turn_status._replace(distance=distance)
                    self.rotate = True
                else:
                    self.get_logger().info('Error unable find direction to forward move robot')

                # self.get_logger().info('Rotate:{} distance:{} raw:{}'.format(angle, distance, self.turn_status))
                self.get_logger().info('Rotate status:{}'.format(self.turn_status))

            else:
                # Move robot forward
                self.twist.linear.x = self.linear + fine_tune_speed
                self.twist.angular.z = 0.0
                if self.fine_tune:
                    self.twist.angular.z = fine_tune_angle
                # debug # - comment out move forward
                self.pub_twist.publish(self.twist)
                self.get_logger().debug('Fine tune angle:{}'.format(fine_tune_angle))

        else:
            # Wati for rotate task complete
            self.get_logger().debug('add roroate start debug code here')


    def rotate_callback(self):
        self.get_logger().info('rotate_callback()')

        while rclpy.utilities.ok():

            if self.rotate == True and self.turn_status.done == False:
                self.get_logger().info("============== Rotate --> START ==============\n")

                # Get the current rotation angle from TF2
                self.odom_angle = self.lidarlib.get_odom_angle(self.tf_buffer, self.odom_frame, self.base_frame)
                self.get_logger().info("ODOM angle={}:{}".format(str(self.odom_angle), str(degrees(self.odom_angle))))
                self.get_logger().info("Turn status:{}".format(self.turn_status))

                last_angle = self.odom_angle
                turn_angle = 0.0
                need_rotate_angle = radians(self.turn_status.turn_angle)

                correction  = need_rotate_angle - turn_angle

                self.get_logger().info("Turn initialize :{}: c:{}:d_c:{}".format(need_rotate_angle, correction, degrees(correction)))

                # self.odom_angular_scale_correction = radians(30)
                self.odom_angular_scale_correction = 1.0

                # 20 -> 0.349
                # 30 -> 0.523
                while abs(correction) > radians(20):
                    if not rclpy.utilities.ok():
                        self.get_lobber().info("rotate thread exit")
                        return

                    # Rotate the robot to reduce the correction angle
                    self.twist.linear.x = 0.0
                    self.twist.angular.z = copysign(self.angular, correction)
                    self.pub_twist.publish(self.twist)
                    time.sleep(0.5)

                    # Get the current rotation agne form tf2
                    self.odom_angle = self.lidarlib.get_odom_angle(self.tf_buffer, self.odom_frame, self.base_frame)
                    self.get_logger().debug("odom_radian={} - {}".format(str(self.odom_angle), str(degrees(self.odom_angle))))

                    # Compute how far we have gone since the last measurement
                    delta_angle = self.odom_angular_scale_correction * self.lidarlib.normalize_angle(self.odom_angle - last_angle)

                    # Add to our total angle so far
                    turn_angle += delta_angle

                    # Compute the new correction
                    correction = need_rotate_angle - turn_angle

                    self.get_logger().debug("odom_angle: {} turn_angle: {} error:{}".format(str(degrees(self.odom_angle)), str(degrees(turn_angle)), str(degrees(correction))))

                    # Store the current angle for the next comparison
                    last_angle = self.odom_angle

                    # Find the front obstacle distance
                    lidar_samples = self.lidarlib.get_lidar_samples()
                    (distance, angle, index) = self.lidarlib.front_lidar_distance(lidar_samples)

                    if abs(distance - float(self.turn_status.distance)) < 0.05:
                        self.get_logger().info("Rotate to expect distance:{} vs current: [d:a]:{}:{}".format(str(self.turn_status.distance), distance, angle))    
                        break

                # Stop the robot
                self.pub_twist.publish(Twist())
                # Update the status flag
                self.rotate = False
                self.turn_status = self.turn_status._replace(done=True)
                self.get_logger().info("============== Rotate --> END ==============\n")
            else:
                if self.rotate == True:
                    self.get_logger().info("Turn status:{}".format(self.turn_status))
                    self.get_logger().info("============== Rotate --> CANCEL ==============\n")
                self.rotate = False
                self.turn_status = self.turn_status._replace(done=True)

            time.sleep(0.1)


def main(args=None):  
    rclpy.init(args=args)

    laser_copilot_node = LaserCopilot()

    try:
        rclpy.spin(laser_copilot_node)
    except KeyboardInterrupt:
        print('\ncontrol-c: laser_avoidance_node shutting down')
    finally:
        # Destroy the node explictly - don't depend on garbage collector
        laser_copilot_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()