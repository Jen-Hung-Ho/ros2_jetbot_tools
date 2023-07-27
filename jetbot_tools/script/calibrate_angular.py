#!/usr/bin/env python3

""" calibrate_angular.py - Version 1.1 2013-12-20
    Rotate the robot 360 degrees to check the odometry parameters of the base controller.
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
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry

# from tf.broadcaster import TransformBroadcaster
from tf2_ros import TransformBroadcaster
# from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from math import radians, copysign, pi, degrees
import PyKDL


class CalibrateAngular(Node):
    
    i = 0
    msg = String()

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'start_test' and param.type_ == Parameter.Type.BOOL:
                self.start_test = param.value
            elif param.name == 'tolerance' and param.type_ == Parameter.Type.DOUBLE:
                self.get_logger().info('tolerance= {}'.format(str(param.value)))
                self.tolerance = radians(param.value)
        return SetParametersResult(successful=True)


    def __init__(self):
        super().__init__('calibrate_node')

        self.hz = 1
        period_fortimer = 1.0 / self.hz

        self.name_space = self.declare_parameter(
            'namespace', 'jetbot_pro').get_parameter_value().string_value

        # How fast will we check the odometry values?
        self.rate = self.declare_parameter(
            'rate', 20.0).get_parameter_value().double_value

        # The test angle is 360 degree
        self.test_angle = radians(self.declare_parameter(
            'test_angle', 360.0).get_parameter_value().double_value)
        self.speed = self.declare_parameter(
            'speed', 1.0).get_parameter_value().double_value
        # self.tolerance = 0.01745329
        self.tolerance  = radians(self.declare_parameter(
            'tolerance', 1.0).get_parameter_value().double_value)
        self.odom_angular_scale_correction = 1.0
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
        self.odom_angle = self.get_odom_angle()
        self.get_logger().info("angle={}".format(str(self.odom_angle)))


    def thread_callback(self):
        self.get_logger().info('Thread callback : {}'.format("hoj") )
        # frequency 1/2  = 0.5 second
        # 1/20 = 0.05
        reverse = 1
        test_off = True

        rate = self.create_rate(self.rate)
        while rclpy.utilities.ok():
            
            if self.start_test:
                test_off = False
                self.get_logger().info("============== Test --> START ==============\n")
                
                # Get the currentrotation angle from tf2
                self.odom_angle = self.get_odom_angle()
                self.get_logger().info("angle={}".format(str(self.odom_angle)))

                last_angle = self.odom_angle
                turn_angle = 0
                self.test_angle *= reverse
                error = self.test_angle - turn_angle

                # Alternate directions between tests
                reverse = -reverse

                while abs(error) > self.tolerance and self.start_test:
                    if not rclpy.utilities.ok():
                        return
                    
                    # Rotate the robot to reduce the error
                    move_cmd = Twist()
                    move_cmd.angular.z = copysign(self.speed, error)
                    self.cmd_vel.publish(move_cmd)
                    # rate.sleep()
                    time.sleep(0.1)

                    # Get the current rotation agne form tf2
                    self.odom_angle = self.get_odom_angle()
                    self.get_logger().info("angle={} - {}".format(str(self.odom_angle), str(degrees(self.odom_angle))))
        
                    # Compute how far we have gone since the last measurement
                    delta_angle = self.odom_angular_scale_correction * self.normalize_angle(self.odom_angle - last_angle)

                    # Add to our total angle so far
                    turn_angle += delta_angle

                    # Compute the new error
                    error = self.test_angle - turn_angle

                    self.get_logger().info("odom_angle: {} turn_angle: {} error:{}".format(str(degrees(self.odom_angle)), str(degrees(turn_angle)), str(degrees(error))))

                    # Store the current angle for the next comparison
                    last_angle = self.odom_angle

                # Stop the robot
                self.cmd_vel.publish(Twist())
                # Update the status flag
                self.start_test = False
                # self.set_parameters()

            else:
                if not test_off:
                    self.get_logger().info("============== Test --> END ==============\n")
                    test_off = True

            rate.sleep()
            # time.sleep(0.2)

    def get_odom_angle(self):
        # Get the current transform between the odom and base frames
        try:
            t = self.tf_buffer.lookup_transform(
                self.odom_frame, 
                self.base_frame, 
                rclpy.time.Time())
            self.get_logger().debug('{}'.format(t.transform.rotation))
        except:
            self.get_logger().info('TF2 transform exception')
            return 0.0
        
        # Convert the rotation from a quaternion to an Euler angle
        return self.quat_to_angle(t.transform.rotation)
    
    def quat_to_angle(self, quat):
        rot = PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w)
        return rot.GetRPY()[2]
    
    def normalize_angle(self, angle):
        res = angle
        while res > pi:
            res -= 2.0 * pi
        while res < -pi:
            res += 2.0 * pi
        return res


def main(args=None):
    rclpy.init(args=args)

    calibrateangle_node = CalibrateAngular()

    try:
        rclpy.spin(calibrateangle_node)
    except KeyboardInterrupt:
        print('\ncontrol-c: caliberateangle_node shutting down')
    finally:
        # Destroy the node explictly - don't depend on garbage collector
        calibrateangle_node.destroy_node()
        rclpy.shutdown()



if __name__ == '__main__':
    
    main()
    
    
