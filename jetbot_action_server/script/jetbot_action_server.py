#!/usr/bin/env python3
#
# Copyright (c) 2026, Jen-Hung Ho 
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
import asyncio
import ast    # Parse string into a 2D array

from rclpy.node import Node

from geometry_msgs.msg import Twist, Point
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
# from collections import namedtuple

from std_msgs.msg import String

from jetbot_action_interface.action import JetbotCommand
from jetbot_tools.include.lidar_utilities import LidarTools
from jetbot_tools.include.tf2_utilities import NamespaceTransformListener, get_tf2_namespace
from jetbot_tools.include.node_parameter_utility import NodeParamTools
from jetbot_tools.include.robot_acton_utility import RobotCommand, RobotActionTool
from jetbot_action_server.include.jetbot_action_controller import JetbotActionController, robot_task

#
#   This robot action server provides a simple interface to control a robot using RobotActionTool utility
#
class RobotCommandActionServer(Node):
    def __init__(self):
        super().__init__('jetbot_command_action_server')

        self.ASR_node = self.declare_parameter('ASR_node', '/Riva_ASR_processor').get_parameter_value().string_value
        self.ASR_topic = self.declare_parameter('ASR_topic', '/voice/transcripts').get_parameter_value().string_value
        self.chat_topic = self.declare_parameter('chat_topic', "/chatbot/response").get_parameter_value().string_value
        self.TTS_topic = self.declare_parameter('TTS_topic', '/jetbot_TTS/transcripts').get_parameter_value().string_value
        self.command_nodes = self.declare_parameter('command_nodes', ["/laser_avoidance","/detect_copilot", "/follow_copilot"]).get_parameter_value().string_array_value
        self.jetbot_tasks = self.declare_parameter('jetbot_tasks', "[['follow','2:follow_detect'],['self-driving', '0:start']]").get_parameter_value().string_value
        # jetbot client cmd
        self.cmd_vel_topic = self.declare_parameter('cmd_vel_topic', '/cmd_vel').get_parameter_value().string_value
        self.global_ns= self.declare_parameter('global_ns', False).get_parameter_value().bool_value
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').get_parameter_value().string_value
        self.base_frame = self.declare_parameter('base_frame', 'base_footprint').get_parameter_value().string_value
        self.angular_tolerance = self.declare_parameter('angular_tolerance', 5.0).get_parameter_value().double_value
        self.linear_tolerance = self.declare_parameter('linear_tolerance', 0.03).get_parameter_value().double_value
        self.odom_angular_scale_correction = self.declare_parameter('odom_angular_scale_correction', 1.0).get_parameter_value().double_value
        self.odom_linear_scale_correction = self.declare_parameter('odom_linear_scale_correction', 1.0).get_parameter_value().double_value

        # Build ASR command to robot action / task mappings
        self.build_ASR_to_action_mappings()


        # Print "RobotCommandActionServer" node parameters
        self.get_logger().info("=========================================")
        self.get_logger().info('Starting Robot Command Action Server')
        self.get_logger().info('ASR_node        :{}'.format(self.ASR_node))
        self.get_logger().info('ASR_topic       :{}'.format(self.ASR_topic))
        self.get_logger().info('chat_topic      :{}'.format(self.chat_topic))
        self.get_logger().info('TTS_topic       :{}'.format(self.TTS_topic))
        self.get_logger().info('command_nodes   :{}'.format(self.command_nodes))
        self.get_logger().info('jetbot_tasks    :{}'.format(self.task_dict_array.keys()))
        # jetbot client cmd
        self.get_logger().info('cmd_vel_topic  :{}'.format(self.cmd_vel_topic))
        self.get_logger().info('tf2_global broadcast- :{}'.format(self.global_ns))
        self.get_logger().info('odom_frame     :{}'.format(self.odom_frame))
        self.get_logger().info('base_frame     :{}'.format(self.base_frame))
        self.get_logger().info('angular_tolerance:{}'.format(self.angular_tolerance))
        self.get_logger().info('linear_tolerance :{}'.format(self.linear_tolerance))
        self.get_logger().info('odom_angular_scale_correction :{}'.format(self.odom_angular_scale_correction))
        self.get_logger().info('odom_linear_scale_correction  :{}'.format(self.odom_linear_scale_correction))
        self.get_logger().info("=========================================")

        # Initialize the stop condition for the action server
        self._should_stop = False

        self.node_param_util = NodeParamTools(self, executor)
        self.mutex = threading.Lock()
        # Creating a single event loop in a background thread and have it service all async needs.
        self.async_loop = asyncio.new_event_loop()
        threading.Thread(target=self.async_loop.run_forever).start()

        # Initialize the tf2 listener
        self.tf_buffer = Buffer()
        if not self.global_ns:
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.lidarlib = LidarTools(self.get_logger())
        else:
            namespace = get_tf2_namespace(self.base_frame)
            self.get_logger().info('namespace : {}'.format(namespace))
            self.tf_listener = NamespaceTransformListener(namespace, self.tf_buffer, self)

        self.pub_twist = self.create_publisher(
            Twist, self.cmd_vel_topic, 10)
        
        self.robot_action = RobotActionTool(
            self.get_logger(), 
            self.lidarlib, 
            self.pub_twist, 
            self.odom_angular_scale_correction, 
            self.odom_linear_scale_correction)
        
        # Add JetbotActionController initialization here
        self.jetbot_action_controller = JetbotActionController(self, self.node_param_util)

        # Creating a single event loop in a background thread and have it service all async needs.
        self.async_loop = asyncio.new_event_loop()
        threading.Thread(target=self.async_loop.run_forever).start()

        self._goal_lock = threading.Lock()
        self._callback_group = ReentrantCallbackGroup()

        self._action_server = ActionServer(
            self,
            JetbotCommand,
            'robot_command',
            self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self._callback_group
        )

    #
    # Build ASR command to robot action / task mappings
    #
    def build_ASR_to_action_mappings(self):

        # Parse the string into a 2D array
        self.task_two_dim_array = ast.literal_eval(self.jetbot_tasks)
        self.task_dict_array = {row[0]: row[1] for row in self.task_two_dim_array}

        # follow_task = robot_task(False, "/follow_copilot", "follow_detect")
        parts = self.task_dict_array['follow'].split(':')
        node_index = int(parts[0])
        follow_task = robot_task(False, self.command_nodes[node_index], parts[1])

        # self_driving_task = robot_task(False, "/laser_avoidance", "start")
        parts = self.task_dict_array['self-driving'].split(':')
        node_index = int(parts[0])
        self_driving_task = robot_task(False, self.command_nodes[node_index], parts[1])

        self.command_dict = {
            'follow': follow_task,
            'self-driving': self_driving_task
        }


    #
    # Remove nodes for get/set parameter service call
    #
    def cleanup(self):
        # Stop the async event loop
        self.async_loop.call_soon_threadsafe(self.async_loop.stop)

        self.node_param_util.cleanup()
        pass

    #
    # Callback for handling goal requests
    #
    def goal_callback(self, goal_request):
        self.get_logger().info(
            f"Received goal request: command={goal_request.command}, "
            f"angle={goal_request.angle}, distance={goal_request.distance}, "
            f"tolerance={goal_request.tolerance}"
        )

        return GoalResponse.ACCEPT

    #
    # Callback for handling cancel requests
    #
    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        # Stop any ongoing robot action
        self.robot_action.cancel_action()
        self.jetbot_action_controller.reset_robot_running_task(self.command_dict["follow"])
        self.jetbot_action_controller.reset_robot_running_task(self.command_dict["self-driving"])
        return CancelResponse.ACCEPT

    #
    # Main callback for executing the command
    #
    async def execute_callback(self, goal_handle):
        # self.get_logger().info(f'Executing command: {goal_handle.request.command}')
        req = goal_handle.request
        self.get_logger().info(
            f"Executing command: command={req.command}, "
            f"angle={req.angle}, distance={req.distance}, "
            f"tolerance={req.tolerance}"
        )

        # Initial feedback
        feedback_msg = JetbotCommand.Feedback()
        feedback_msg.status = "Started"
        feedback_msg.progress = 0.0
        goal_handle.publish_feedback(feedback_msg)

        result = JetbotCommand.Result()
        success = False

        # Reset the stop condition
        self._should_stop = False

        command = req.command.lower()
        if command == "move":
            move_status = RobotCommand(False, 0, req.distance, self.linear_tolerance)
            self.robot_action.move_robot(self.tf_buffer, self.odom_frame, self.base_frame, move_status)
            success = True
            result.message = "Move command executed."
        elif command == "turn":
            turn_status = RobotCommand(False, req.angle, 0.0, self.angular_tolerance)
            self.robot_action.rotate_robot(self.tf_buffer, self.odom_frame, self.base_frame, turn_status)
            success = True
            result.message = "Turn command executed."
        elif command == "stop":
            # Stop the robot and reset the action state
            self._should_stop = True
            self.robot_action.cancel_action()
            self.jetbot_action_controller.reset_robot_running_task(self.command_dict["follow"])
            self.jetbot_action_controller.reset_robot_running_task(self.command_dict["self-driving"])
            success = True
            result.message = "Stop command executed."
        elif command == "follow":
            # follow_task = robot_task(False, "/follow_copilot", "follow_detect")
            follow_task = self.command_dict["follow"]
            success = self.jetbot_action_controller.robot_follow(goal_handle, follow_task)
        elif command == "self-driving":
            # self_driving_task = robot_task(False, "/laser_avoidance", "start")
            self_driving_task = self.command_dict["self-driving"]
            success = self.jetbot_action_controller.robot_self_driving(goal_handle, self_driving_task)
        else:
            result.message = f"Unknown command: {req.command}"
            self.get_logger().warn(result.message)

        TTS_string = String()
        TTS_string.data = "{} command complete".format(command)
        self.jetbot_action_controller.pub_chat.publish(TTS_string)

        if success:
            goal_handle.succeed()
            result.success = True
        else:
            goal_handle.abort()
            result.success = False

        return result

def main(args=None):
    rclpy.init(args=args)

    # Use a MultiThreadedExecutor to handle incoming goal requests concurrently
    global executor 
    executor = rclpy.executors.MultiThreadedExecutor()

    node = RobotCommandActionServer()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt received, shutting down...')
    finally:
        # stops the executor and ensures that no more callbacks are processed.
        executor.shutdown()
        node.cleanup()
        node.destroy_node()

        # rclpy.spin(node)
        rclpy.shutdown()

if __name__ == '__main__':
    main()