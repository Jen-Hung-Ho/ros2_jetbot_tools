#!/usr/bin/env python3

""" 
    tf2_listener_copilot.py reference list:

    https://github.com/ros/geometry_tutorials/tree/ros2/turtle_tf2_py/turtle_tf2_py
    https://automaticaddison.com/how-to-create-a-tf-listener-using-ros-2-and-python/
"""

import rclpy
import math
import threading
import time

from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class CalibrateLinear(Node):
    
    i = 0
    msg = String()

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'start_test' and param.type_ == Parameter.Type.BOOL:
                self.start_test = param.value
                self.get_logger().info('start_test= {}'.format(bool(param.value)))
            elif param.name == 'tolerance' and param.type_ == Parameter.Type.DOUBLE:
                self.get_logger().info('tolerance= {}'.format(str(param.value)))
                self.tolerance = param.value
        return SetParametersResult(successful=True)


    def __init__(self):
        super().__init__('jetbot_follower')

        self.hz = 1
        period_fortimer = 1.0 / self.hz

        self.follower = self.declare_parameter( 'follower', 'GoPiGo3').get_parameter_value().string_value

        # How fast will we check the odometry values?
        self.rate = self.declare_parameter(
            'rate', 20.0).get_parameter_value().double_value
        
        self.scale_foward_speed = self.declare_parameter(
            'speed', 0.5).get_parameter_value().double_value
        self.tolerance  = self.declare_parameter(
            'tolerance', 0.01).get_parameter_value().double_value
        
        self.scale_rotation_rate = self.declare_parameter(
            'scale_rotation_rate', 1.0).get_parameter_value().double_value
        self.start_test = self.declare_parameter(
            'start_test', False).get_parameter_value().bool_value

        # Declare and acuqire 'target_frame' parameter
        self.to_frame = self.declare_parameter(
            'to_frame', 'GoPiGo3/base_footprint').get_parameter_value().string_value

        self.from_frame = self.declare_parameter(
            'from_frame', 'jetbot1/base_footprint').get_parameter_value().string_value
        
        # Add parameters callback 
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        self.get_logger().info('from frame: {}, to_frame: {}'.format(str(self.from_frame), str(self.to_frame)))
        self.publisher = self.create_publisher(String, '/' + self.follower + '/chatter', 10)
        self.cmd_vel = self.create_publisher(Twist, '/' + self.follower + '/cmd_vel', 20)

        # Initialize the tf2 listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)


        self.thread = threading.Thread(target=self.thread_callback)
        self.thread.start()



    def thread_callback(self):
        self.get_logger().info('Thread callback : {}'.format("TF2 listener") )
    
        # Get the starting position from the tf2 transform between the 2 robots base_link frames
        self.position = self.get_position()
        self.get_logger().info("Init TF move=[{:.2f} : {:.2f} : {:.2f}]".format(self.position.x, self.position.y, self.position.z))

        rate = self.create_rate(self.rate)
        while rclpy.utilities.ok():
            
            # Stop the robot by default
            move_cmd = Twist()

            if self.start_test: 
                # Get the current position from the tf2 transform between the odom and base frames
                self.position = self.get_position()
                self.get_logger().debug("Move={}".format(str(self.position)))
                
                # Correct the estimated distance by the correction factor
                distance = self.scale_foward_speed * math.sqrt(
                    math.pow(self.position.x, 2) +
                    math.pow(self.position.y, 2))
                
                if distance > self.tolerance:
                    move_cmd.linear.x = distance
                    # Compput the rotate angle
                    move_cmd.angular.z = self.scale_rotation_rate * math.atan2(
                        self.position.y,
                        self.position.x)


                    self.get_logger().debug("Twist={}".format(str(move_cmd)))
                    self.get_logger().info("Move=[{:.2f} : {:.2f} : {:.2f}]".format(self.position.x, self.position.y, self.position.z))

                self.cmd_vel.publish(move_cmd)
                # self.get_logger().info("move={}".format(str(move_cmd)))

            else:
                # self.position = self.get_position()
                # self.get_logger().debug("Init postion={}".format(str(self.position)))
                self.get_logger().debug("Init transformation=[{:.2f} : {:.2f} : {:.2f}]".format(self.position.x, self.position.y, self.position.z))

            # self.cmd_vel.publish(move_cmd)
            rate.sleep()
            # time.sleep(0.2)

    def get_position(self):
        # Get the current transform between the odom and base frames
        try:
            t = self.tf_buffer.lookup_transform(
                self.to_frame, 
                self.from_frame, 
                rclpy.time.Time())
            self.get_logger().debug('get_position:{}'.format(t.transform.translation))
            # self.get_logger().info('get_position:{}'.format(t.transform.translation))
        except TransformException as ex:
            self.get_logger().info(f'TF2 transform exception {self.to_frame} to {self.from_frame}: {ex}')
            return Point()
        
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
    
    
