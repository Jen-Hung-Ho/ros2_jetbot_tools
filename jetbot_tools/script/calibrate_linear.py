#!/usr/bin/env python3

""" calibrate_linear.py - Version 1.1 2013-12-20
    Move the robot 1.0 meter to check on the PID parameters of the base controller.
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2012 Patrick Goebel.  All rights reserved.
    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
      
"""

import rclpy
import threading
import time

from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
# from rclpy.utilities import ok
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point, Vector3Stamped
from nav_msgs.msg import Odometry

# from tf.broadcaster import TransformBroadcaster
from tf2_ros import TransformBroadcaster
# from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from math import copysign, sqrt, pow
import PyKDL


class CalibrateLinear(Node):
    
    i = 0
    msg = String()

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'start_test' and param.type_ == Parameter.Type.BOOL:
                self.start_test = param.value
            elif param.name == 'tolerance' and param.type_ == Parameter.Type.DOUBLE:
                self.get_logger().info('tolerance= {}'.format(str(param.value)))
                self.tolerance = param.value
        return SetParametersResult(successful=True)


    def __init__(self):
        super().__init__('calibrate_linear')

        self.hz = 1
        period_fortimer = 1.0 / self.hz

        self.name_space = self.declare_parameter( 'namespace', 'jetbot_pro').get_parameter_value().string_value

        # How fast will we check the odometry values?
        self.rate = self.declare_parameter(
            'rate', 20.0).get_parameter_value().double_value
        # Set the distance to traver meters
        self.test_distance = self.declare_parameter(
            'test_distance', 1.0).get_parameter_value().double_value
        self.speed = self.declare_parameter(
            'speed', 0.3).get_parameter_value().double_value
        # self.tolerance = 0.03 meters
        self.tolerance  = self.declare_parameter(
            'tolerance', 0.03).get_parameter_value().double_value
        # self.odom_linear_scale_correction = 1.0
        self.odom_linear_scale_correction = self.declare_parameter(
            'odom_linear_scale_correction', 1.0).get_parameter_value().double_value
        self.start_test = self.declare_parameter(
            'start_test', False).get_parameter_value().bool_value

        # Declare and acuqire 'target_frame' parameter
        self.base_frame = self.declare_parameter(
            'base_frame', 'base_footprint').get_parameter_value().string_value
        self.base_frame = self.name_space + '/' + self.base_frame

        self.odom_frame = self.declare_parameter(
            'odom_frame', 'odom').get_parameter_value().string_value
        
        # Add parameters callback 
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        self.get_logger().info('odem frame: {}, base_frame: {}'.format(str(self.odom_frame), str(self.base_frame)))
        self.publisher = self.create_publisher(String, '/' + self.name_space + '/chatter', 10)
        self.cmd_vel = self.create_publisher(Twist, '/' + self.name_space + '/cmd_vel', 20)

        # Initialize the tf2 listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # self.timer = self.create_timer( period_fortimer, self.timer_callback)
        # self.get_logger().info('caliberate_node: created main loop call back at {} Hz'.format(self.hz))

        self.thread = threading.Thread(target=self.thread_callback)
        self.thread.start()

    
    def timer_callback(self):
        self.msg.data =  'Hello World: %d' % self.i
        self.i += 1
        # self.get_logger().info('Main callback Publishing: "%s"' % self.msg.data)
        # self.publisher.publish(self.msg)
        self.position = self.get_position()
        self.get_logger().info("position={}".format(str(self.position)))


    def thread_callback(self):
        self.get_logger().info('Thread callback : {}'.format("hoj") )
        # frequency 1/2  = 0.5 second

        test_off = True

        # Get the starting position from the tf2 transform between the odom and base frames
        self.position = self.get_position()
        self.get_logger().info("postion={}".format(str(self.position)))

        x_start = self.position.x
        y_start = self.position.y

        rate = self.create_rate(self.rate)
        while rclpy.utilities.ok():
            
            # Stop the robot by default
            move_cmd = Twist()

            if self.start_test:

                if test_off:
                    self.get_logger().info("============== Test --> START ==============\n")
                    test_off = False
                
                # Get the current position from the tf2 transform between the odom and base frames
                self.position = self.get_position()
                # self.get_logger().info("postion={}".format(str(self.position)))

                # Compute the Euclidean distance from the target point
                distance = sqrt(pow((self.position.x - x_start), 2) +
                                pow((self.position.y - y_start), 2))

                # Correct the estimated distance by the correction factor
                distance *= self.odom_linear_scale_correction

                # How close are we?
                error =  distance - self.test_distance

                self.get_logger().info("position: {} distance: {} error:{}".format(str(self.position), str(distance), str(error)))
                
                # Are we close enough?
                if not self.start_test or abs(error) <  self.tolerance:
                    test_off = True
                    self.start_test = False
                    move_cmd = Twist()
                    self.get_logger().info("============== Test --> END ==============\n")
                else:
                    # If not, move in the appropriate direction
                    move_cmd.linear.x = copysign(self.speed, -1 * error)

                self.cmd_vel.publish(move_cmd)

            else:
                self.position = self.get_position()
                x_start = self.position.x
                y_start = self.position.y

            # self.cmd_vel.publish(move_cmd)
            rate.sleep()
            # time.sleep(0.2)

    def get_position(self):
        # Get the current transform between the odom and base frames
        try:
            t = self.tf_buffer.lookup_transform(
                self.odom_frame, 
                self.base_frame, 
                rclpy.time.Time())
            self.get_logger().debug('get_position:{}'.format(t.transform.translation))
            # self.get_logger().info('get_position:{}'.format(t.transform.translation))
        except:
            self.get_logger().info('TF2 transform exception')
            return Point()
        
        # Convert the rotation from a quaternion to an Euler angle
        # return Point(t.transform.translation)
        p = Point()
        p.x = t.transform.translation.x
        p.y = t.transform.translation.y
        p.z = t.transform.translation.z
        return p
    
    


def main(args=None):
    rclpy.init(args=args)

    calibratelinear_node = CalibrateLinear()

    try:
        rclpy.spin(calibratelinear_node)
    except KeyboardInterrupt:
        print('\ncontrol-c: caliberateangle_node shutting down')
    finally:
        # Destroy the node explictly - don't depend on garbage collector
        calibratelinear_node.destroy_node()
        rclpy.shutdown()



if __name__ == '__main__':
    
    main()
    
    
