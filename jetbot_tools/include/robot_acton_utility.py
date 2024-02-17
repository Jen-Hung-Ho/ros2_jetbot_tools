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

import rclpy
import threading
import time

from collections import namedtuple
from geometry_msgs.msg import Twist, Point
from math import radians, copysign, degrees, sqrt, pow

# Define the namedtuple at the module level
RobotCommand = namedtuple('command', ['done', 'turn_angle', 'distance', 'delta'])

#
# This class provide simple ROS robot move actions
#
class RobotActionTool():
    def __init__(self, logger, lidarlib, pub_twist, angular_scale_correction, linear_scale_correction):

        self.twist = Twist()
        self.angular = 0.2
        self.odom_angular_scale_correction = angular_scale_correction
        self.odom_linear_scale_correction = linear_scale_correction
        self.logger = logger
        self.lidarlib = lidarlib
        self.pub_twist = pub_twist
        self.mutex = threading.Lock()
        self.cancel = False

        logger.info('RobotActionTool() initialize')

    #
    #   cancel current running  action
    #
    def cancel_action(self):
        self.cancel = True


    #
    # asynchronous rotate robot command
    #
    async def async_rotate_robot(self, tf_buffer, odom_frame, base_frame, command_status):
        self.logger.info("asynchronous rotate_robot()")
        self.rotate_robot(tf_buffer, odom_frame, base_frame, command_status)


    #
    # rotate robot command
    #
    def rotate_robot(self, tf_buffer, odom_frame, base_frame, command_status):
        self.logger.info("============== Rotate --> START ==============\n")

        self.cancel = False

        # turn_status = self.command_dict['turn']
        turn_status = command_status

        # Get the current rotation angle from TF2
        self.odom_angle = self.lidarlib.get_odom_angle(tf_buffer, odom_frame, base_frame)
        self.logger.info("ODOM angle={}:{}".format(str(self.odom_angle), str(degrees(self.odom_angle))))
        self.logger.info("Turn status:{}".format(turn_status))

        last_angle = self.odom_angle
        turn_angle = 0.0
        need_rotate_angle = radians(turn_status.turn_angle)

        correction  = need_rotate_angle - turn_angle

        self.logger.info("Turn initialize :{}: c:{}:d_c:{}".format(need_rotate_angle, correction, degrees(correction)))

        # 20 -> 0.349
        # 30 -> 0.523
        while abs(correction) > radians(turn_status.delta):
            if not rclpy.utilities.ok():
                self.logger.info("rotate_robot() thread exit")
                return
            elif self.cancel:
                self.logger.info("============== Rotate --> CANCEL ==============\n")
                self.cancel = False
                break

            # Rotate the robot to reduce the correction angle
            self.twist.linear.x = 0.0
            self.twist.angular.z = copysign(self.angular, correction)
            self.pub_twist.publish(self.twist)
            time.sleep(0.5)

            # Get the current rotation agne form tf2
            self.odom_angle = self.lidarlib.get_odom_angle(tf_buffer, odom_frame, base_frame)
            self.logger.debug("odom_radian={:.2f} - {:.2f}".format(self.odom_angle, degrees(self.odom_angle)))

            # Compute how far we have gone since the last measurement
            delta_angle = self.odom_angular_scale_correction * self.lidarlib.normalize_angle(self.odom_angle - last_angle)

            # Add to our total angle so far
            turn_angle += delta_angle

            # Compute the new correction
            correction = need_rotate_angle - turn_angle

            self.logger.info("odom_angle: {:.2f} turn_angle: {:.2f} error:{:.2f}".format(degrees(self.odom_angle), degrees(turn_angle), degrees(correction)))

            # Store the current angle for the next comparison
            last_angle = self.odom_angle

        # Stop the robot
        self.pub_twist.publish(Twist())
        self.logger.info("============== Rotate --> END ==============\n")

    #
    # asynchronous move robot command
    #
    async def async_move_robot(self, tf_buffer, odom_frame, base_frame, command_status):
        self.logger.info("asynchronous move_robot()")
        self.move_robot(tf_buffer, odom_frame, base_frame, command_status)


    #
    # move robot command
    #
    def move_robot(self, tf_buffer, odom_frame, base_frame, command_status):
        self.logger.info("============== Move --> START ==============\n")

        self.cancel = False

        move_status = command_status
        move_distance = float(move_status.distance)

        # self.odom_linear_scale_correction = 1.0
        self.speed = 0.3
        move_cmd = Twist()
        
        # Get the start positon from TF2
        self.position = self.get_position(tf_buffer, odom_frame, base_frame)
        x_start = self.position.x
        y_start = self.position.y
        # Get initialize Euclidean distance from the target point
        error = move_distance

        # self.get_logger().info("postion={}".format(str(self.position)))
        self.logger.info("Move status:{}".format(move_status))

        # Stop wthen moveing distance reach the expected distance
        while abs(error) >  move_status.delta:
            if not rclpy.utilities.ok():
                self.logger.info("move_robot() thread exit")
                return
            elif self.cancel:
                self.logger.info("============== Move --> CANCEL ==============\n")
                self.cancel = False
                break

            # Get the current positon from TF2
            self.position = self.get_position(tf_buffer, odom_frame, base_frame)

            # Compute the Euclidean distance from the target point
            distance = sqrt(pow((self.position.x - x_start), 2) +
                            pow((self.position.y - y_start), 2))

            # Correct the estimated distance by the correction factor
            distance *= self.odom_linear_scale_correction

            # How close are we?
            if move_distance > 0:
                error = distance - move_distance
            else:
                error = distance + move_distance

            self.logger.info("position: [x= {:.2f}, y= {:.2f}, z= {:.2f}] distance: {:.2f} error: {:.2f}"\
                                   .format(self.position.x, self.position.y, self.position.z,  distance, error))

            # Move in the appropriate direction
            move_cmd.linear.x = copysign(self.speed, move_distance)

            self.pub_twist.publish(move_cmd)
            time.sleep(0.2)

        # Stop the robot
        self.pub_twist.publish(Twist())
        self.logger.info("============== Move --> END ==============\n")


    #
    # Get robot ODOM translation postion
    #
    def get_position(self, tf_buffer, odom_frame, base_frame):
        # Get the current transform between the odom and base frames
        try:
            t = self.lidarlib.get_odom_transform(tf_buffer, odom_frame, base_frame)
            self.logger.debug('get_position:{}'.format(t.transform.translation))
            # self.get_logger().info('get_position:{}'.format(t.transform.translation))
        except:
            self.logger().info('TF2 transform exception')
            return Point()

        # Convert the rotation from a quaternion to an Euler angle
        # return Point(t.transform.translation)
        p = Point()
        p.x = t.transform.translation.x
        p.y = t.transform.translation.y
        p.z = t.transform.translation.z
        return p
