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


import rclpy  # Python library for ROS 2
import threading
# import time
import asyncio
import ast    # Parse string into a 2D array


from rclpy.node import Node # Handles the creation of nodes
# Action client for sending commands to the robot command action server
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterType, SetParametersResult


from std_msgs.msg import String

from collections import namedtuple

from jetbot_tools.include.robot_acton_utility import RobotCommand, RobotActionTool
from jetbot_tools.include.node_parameter_utility import NodeParamTools


from jetbot_action_interface.action import JetbotCommand 

#
# Jetbot tools voice command copiliot
#
class JetbotToolCopilot(Node):

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'start' and param.type_ == Parameter.Type.BOOL:
                self.start = param.value
                self.get_logger().info('start= {}'.format(bool(param.value)))
            elif param.name == 'command' and param.type_ == Parameter.Type.STRING:
                self.cmd = param.value
                self.get_logger().info('command= {}'.format(str(param.value)))
                self.invoke_command(self.cmd)

        return SetParametersResult(successful=True)

    def __init__(self):
        super().__init__('Jetbot_tool_voice_copilot')

        self._action_client = ActionClient(self, JetbotCommand, '/robot_command')

        self.cmd = self.declare_parameter('command', 'jetbot').get_parameter_value().string_value
        self.start = self.declare_parameter('start', True).get_parameter_value().bool_value
        self.TTS_enable = self.declare_parameter('TTS_enable', False).get_parameter_value().bool_value
        self.ASR_node = self.declare_parameter('ASR_node', '/Riva_ASR_processor').get_parameter_value().string_value
        self.ASR_topic = self.declare_parameter('ASR_topic', '/jetbot_voice/transcripts').get_parameter_value().string_value
        self.TTS_topic = self.declare_parameter('TTS_topic', '/jetbot_TTS/transcripts').get_parameter_value().string_value
        self.chat_topic = self.declare_parameter('Response_topic', '/chatbot/response').get_parameter_value().string_value
        self.jetbot_commands = self.declare_parameter('jetbot_commands', "[['left','turn:90'],['right','turn:-90']]").get_parameter_value().string_value
        self.jetbot_tasks = self.declare_parameter('jetbot_tasks', "['follow','2:follow_detect']").get_parameter_value().string_value
        # jetbot client cmd
        self.angular_tolerance = self.declare_parameter('angular_tolerance', 5.0).get_parameter_value().double_value
        self.linear_tolerance = self.declare_parameter('linear_tolerance', 0.03).get_parameter_value().double_value


        # Build ASR command to robot action / task mappings
        self.build_ASR_to_action_mappings()

        # Print "Jetbot_tool_voice_copilot" node parameters
        self.get_logger().info('start           :{}'.format(self.start))
        self.get_logger().info('command         :{}'.format(self.cmd))
        self.get_logger().info('TTS_enable      :{}'.format(self.TTS_enable))
        self.get_logger().info('ASR_node        :{}'.format(self.ASR_node))
        self.get_logger().info('ASR_topic       :{}'.format(self.ASR_topic))
        self.get_logger().info('TTS_topic       :{}'.format(self.TTS_topic))
        self.get_logger().info('chat_topic      :{}'.format(self.chat_topic))
        self.get_logger().info('jetbot_commands :{}'.format(self.cmd_dict_array.keys()))
        self.get_logger().info('jetbot_tasks    :{}'.format(self.task_dict_array.keys()))
        self.get_logger().info('angular_tolerance:{}'.format(self.angular_tolerance))
        self.get_logger().info('linear_tolerance :{}'.format(self.linear_tolerance))

        # Add parameters callback 
        self.add_on_set_parameters_callback(self.parameter_callback)

        # self.init_ros_nodes()
        self.node_param_util = NodeParamTools(self, executor)
        self.mutex = threading.Lock()

        # Create the subscriber. This subscriber will receive ASR message
        self.subscription = self.create_subscription(
            String, 
            self.ASR_topic,
            self.ASR_callback,
            10)

        # Create the publiser to TTS
        self.pub_TTS = self.create_publisher(
            String,
            self.TTS_topic,
            10
        )

        # Create the publisher to chatbot response topic
        self.pub_chatbot = self.create_publisher(
            String, 
            self.chat_topic,
            10
        )

        # Creating a single event loop in a background thread and have it service all async needs.
        self.async_loop = asyncio.new_event_loop()
        threading.Thread(target=self.async_loop.run_forever).start()


    #
    # Build ASR command to robot action / task mappings
    #
    def build_ASR_to_action_mappings(self):
        # Parse the string into a 2D array
        self.cmd_two_dim_array = ast.literal_eval(self.jetbot_commands)
        self.cmd_dict_array = {row[0]: row[1] for row in self.cmd_two_dim_array}
        self.task_two_dim_array = ast.literal_eval(self.jetbot_tasks)
        self.task_dict_array = {row[0]: row[1] for row in self.task_two_dim_array}

    #
    # Remove nodes for get/set parameter service call
    #
    def cleanup(self):
        # Stop the async event loop
        self.async_loop.call_soon_threadsafe(self.async_loop.stop)

        self.node_param_util.cleanup()
        pass


    #
    # NVIDIA jetson-voice ASR ROS2 topic subscription
    #
    def ASR_callback(self, msg):
        self.get_logger().info('ASR: "%s"' % msg.data)


    #
    # Mute ASR processor and wait until LLM chat reponse to TTS task complete
    #
    def mute_ASR_processor(self, node_name):
        # Turn off ASR and wait until LLM chat response to TTS task complete
        passfail, value = self.node_param_util.try_get_node_parameters(node_name, 'start')
        if passfail == True:
            self.get_logger().info('Jetbot chat node start:{}'.format(value.bool_value))
            if value.bool_value:
                self.node_param_util.try_set_node_parameters(node_name, 'start', type=ParameterType.PARAMETER_BOOL, value=False)
        else:
            self.get_logger().info('Jetbot chat {} node not exit, skip this task'.format(node_name))

    #
    # Invoke from ROS2 node parameter_callback()
    #
    def invoke_command(self, cmd):
        self.get_logger().info('Cmd: {}'.format(cmd))
        TTS_echo = False

        # start == false --> ignore jetbot ASR client request
        if not self.start:
            self.get_logger().info("invoke_command() ignore command:{}".format(cmd))
            return

        # Create a JetbotCommand.Goal object to send to the jetbot action server
        jetbot_goal = JetbotCommand.Goal()


        # update self.cmd after done with this command
        # if cmd in self.jetbot_commands:
        if cmd in self.cmd_dict_array.keys():

            if self.TTS_enable:
                TTS_echo = True
    
            # Retrieve node name and command value [command:value]
            parts = self.cmd_dict_array[cmd].split(':')
            self.get_logger().info("jetbot-commands:{} --> {}".format(parts[0], parts[1]))
            if parts[0] == 'turn':
                rotate_angle = float(parts[1])
                jetbot_goal.command = parts[0]
                jetbot_goal.angle = rotate_angle
                jetbot_goal.distance = 0.0
                jetbot_goal.tolerance = self.angular_tolerance
            elif parts[0] == 'move':
                move_distance = float(parts[1])
                self.get_logger().info("move distance :{}".format(move_distance))
                jetbot_goal.command = parts[0]
                jetbot_goal.angle = 0.0
                jetbot_goal.distance = move_distance
                jetbot_goal.tolerance = self.linear_tolerance
            elif parts[0] == 'stop':
                movie_distance = 0;
                self.get_logger().info("stop robot")
                jetbot_goal.command = parts[0]
                jetbot_goal.angle = 0.0
                jetbot_goal.distance = 0.0
                jetbot_goal.tolerance = 0.0

        elif cmd in self.task_dict_array.keys():

            if self.TTS_enable:
                TTS_echo = True

            # Retrieve node index and parameter name [node_index:parameter_name]
            parts = self.task_dict_array[cmd].split(':')
            node_index = int(parts[0])
            # self.get_logger().info("jetbot_tasks: {} --> {}".format(self.command_nodes[node_index], parts[1]))
            if cmd == 'follow':
                jetbot_goal.command = cmd
                jetbot_goal.angle = 0.0
                jetbot_goal.distance = 0.0
                jetbot_goal.tolerance = 0.0
            elif cmd == 'self-driving':
                jetbot_goal.command = cmd
                jetbot_goal.angle = 0.0
                jetbot_goal.distance = 0.0
                jetbot_goal.tolerance = 0.0

        if TTS_echo == True:
            # ASR_client node mute ASR_node before invokde the command parameter
            self.mute_ASR_processor(self.ASR_node)
            # Publish to TTS node to play audio streaming
            TTS_string = String()
            TTS_string.data = "Jetbot process: " + cmd
            self.pub_TTS.publish(TTS_string)
            self.get_logger().info("Publish to TTS topic: {}".format(TTS_string.data))

        # TODO: how to check if the command is not in the list and cancel the existing task before send goal
        # Send goal to the jetbot action server
        self.send_goal(jetbot_goal)


    #
    #  Send goal to the jetbot action server
    #
    def send_goal(self, goal_msg):

        if self._action_client.server_is_ready():
            # Send a JetbotCommand.Goal object as the action goal
            # goal_msg = JetbotCommand.Goal()
            # goal_msg.command = command_str

            self._action_client.wait_for_server()
            self._send_goal_future = self._action_client.send_goal_async(
                goal_msg,
                feedback_callback=self.feedback_callback
            )
            self._send_goal_future.add_done_callback(self.goal_response_callback)
        else:
            msg = 'Jetbot action server is not ready'
            self.get_logger().info(msg)

            # ASR_client node mute ASR_node before publish to chatbot topic
            # Unmute ASR is handle by the Jetbot action server side but it is not ready yet
            self.mute_ASR_processor(self.ASR_node)
            # Publish to chatbot topic to play audio streaming and unute ASR_node after TTS task complete
            TTS_string = String()
            TTS_string.data = msg
            self.pub_chatbot.publish(TTS_string)
            self.get_logger().info("Publish to chatbot topic: {}".format(TTS_string.data))


    #
    # Callback for the goal response from the jetbot action server
    #
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    #
    # Callback for the result from the jetbot action server
    #
    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result}')
        # rclpy.shutdown()

    #
    # Callback for feedback from the jetbot action server
    #
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback}')


    #
    # Clean up running task before start a new one
    #
    def reset_running_tasks(self, command):
        self.robot_action.cancel_action()
        self.reset_robot_running_task('self-driving', command=command)
        self.reset_robot_running_task('follow', command=command)



def main(args=None):
    
    rclpy.init(args=args)

    global executor

    executor = rclpy.executors.MultiThreadedExecutor()
    JetbotToolCopilit_node = JetbotToolCopilot()
    executor.add_node(JetbotToolCopilit_node)

    try:
        # rclpy.spin(JetbotToolCopilit_node)
        executor.spin()
    except KeyboardInterrupt:
        print('\ncontrol-c: JetbotToolCopilit_node shutting down')
    finally:
        # Destroy the node explictly - don't depend on garbage collector
        JetbotToolCopilit_node.cleanup()
        JetbotToolCopilit_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()