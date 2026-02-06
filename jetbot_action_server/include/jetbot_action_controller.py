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
import time
import asyncio
import binascii

from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterType, SetParametersResult
from vision_msgs.msg import Detection2DArray


# TODO clean up not use reference later
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan

from math import radians, degrees
from collections import namedtuple

# jetbot_tool_copilot.py
from jetbot_tools.include.node_parameter_utility import NodeParamTools
# from jetbot_tools.include.detectnet_result_utility import DetectNetResult
from jetbot_tools.include.robot_acton_utility import RobotCommand, RobotActionTool

# Define the namedtuple at the model level
robot_task = namedtuple('task', ['run', 'node_name', 'command'])

class JetbotActionController():

    def __init__(self, node, node_param_util):

        self.node = node
        self.logger = self.node.get_logger()

        self.logger.info('JetbotActionController started.')

        self.node_param_util = node_param_util
        # Initialize the detectnet for robot_follow action
        # self.detect_net_util = DetectNetResult(self.node, self.node_param_util, self.node.node_name, self.node.class_labels, self.node.FoV)

        # Parameter for lisar and detectnet data collection
        self.event = threading.Event()
        self.detect_target = False

        # Create the subscriber. This subscriber will receive ASR message
        self.subscription = self.node.create_subscription(
            String, 
            self.node.ASR_topic,
            self.ASR_callback,
            10)

        # Create the publiser to chat topic - unumute ASR processor
        self.pub_chat = self.node.create_publisher(
            String,
            self.node.chat_topic,
            10
        )

        # Create the publiser to TTS
        self.pub_TTS = self.node.create_publisher(
            String,
            self.node.TTS_topic,
            10
        )


    #
    # NVIDIA jetson-voice ASR ROS2 topic subscription
    #
    def ASR_callback(self, msg):
        self.logger.info('ASR: "%s"' % msg.data)


    #
    # Mute ASR processor and wait until LLM chat reponse to TTS task complete
    #
    def mute_ASR_processor(self, node_name):
        # Turn off ASR and wait until LLM chat response to TTS task complete
        passfail, value = self.node_param_util.try_get_node_parameters(node_name, 'start')
        if passfail == True:
            self.logger.info('Jetbot node {} start:{}'.format(node_name, value.bool_value))
            if value.bool_value == True:
                # Set ASR processor start parameter to false to mute ASR processing
                self.node_param_util.try_set_node_parameters(node_name, 'start', type=ParameterType.PARAMETER_BOOL, value=False)
        else:
            self.logger.info('Jetbot {} node not exit, skip this task'.format(node_name))


    #
    # This method is thread-safe and is intended to be called from a different thread than the one where the event loop is running.
    #
    def thread_safe_async_method(self, coroutine):
        # loop = asyncio.get_event_loop()
        # Schedules a coroutine to be run in the event loop.
        # task = asyncio.run_coroutine_threadsafe(coroutine, loop)

        # Use the event loop created in the main node thread
        task = asyncio.run_coroutine_threadsafe(coroutine,  self.node.async_loop)
        # Returns a concurrent.futures.Future object representing the execution of the coroutine.
        return task

    #
    # robot follow task -> Jetbot ROS2 follow_copilot node
    #
    def robot_follow(self, goal_handle, follow_task):
        self.logger.info("robot_follow")
        pass

        # follow_status = self.command_dict['follow']
        self.logger.info("Robot follow status:{}".format(follow_task))
        self.detect_model_ready = False

        # robot follow detection procedure
        # 1) Ensure the follow copilot node exist
        # 2) Stop laser_avoidance and follow_copilot node if is is in running state
        # 3) Turn on the detectnet detection data collecting flag to look fo target vision
        # 4) Start follow_copilot node to start fobot follow task

        # Step 1) Ensure the follow_copilot ROS2 node exist
        # ros2 param get /follow_copilot follow_detect -- false
        # node_index = follow_status.node_index
        node_name = follow_task.node_name
        command = follow_task.command
        passfail = self.node_param_util.node_exists(node_name)
        # passfail, value = self.node_param_util.try_get_node_parameters(node_name, command)
        if passfail == False:
            self.mute_ASR_processor(self.node.ASR_node)
            TTS_string = String()
            TTS_string.data = "follow_copilot node not exist, skip the task"
            self.pub_TTS.publish(TTS_string)
            self.logger.info(TTS_string.data)
            return False
        else:
            self.logger.info("{} node exist, continue the task".format(node_name))
            passfail, value = self.node_param_util.try_get_node_parameters(node_name, 'class_label_names')
            if passfail:
                class_label_names = value.string_array_value
                if isinstance(class_label_names, (list, tuple)) and len(class_label_names) > 1:
                    self.logger.info(f'class_label_names array size: {len(class_label_names)}')
                    self.detect_model_ready = True
            else:
                self.logger.info("follow_copilot class_label_names parameter not exist")
                return False

        # Step 3) Turn on the detectnet detection data collecting flag to look fo target vision
        self.event.clear()


        TTS_string = String()
        if self.detect_model_ready:
            self.logger.info('Detect person model ready, start collecting detectnet data for follow task')
            self.collect_detectnet_data = 0
            self.node.robot_action.cancel_action()
            # time.sleep(0.5)

            TTS_string.data = "AI person detect model ready to start following task"
            self.logger.info("xxxxxxx {} xxxxxxx".format(TTS_string.data))
        else:
            self.node.robot_action.cancel_action()
            TTS_string.data = "TIMEOUT: Failed to detect person model, skip the follow task"
            self.logger.error("{}".format(TTS_string.data))
        self.mute_ASR_processor(self.node.ASR_node)
        self.pub_TTS.publish(TTS_string)

        # Check for cancellation before starting the task
        if goal_handle.is_cancel_requested:
            self.logger.info("Goal handle is cancel requested, stop the self driving task")
            return False

        uuid_bytes = goal_handle.goal_id.uuid.tobytes()
        uuid_hex = binascii.hexlify(uuid_bytes).decode('utf-8')
        self.logger.info("Goal nandle ID:{} command: {}".format(uuid_hex, goal_handle.request.command))

        # Step 6) Start follow_copilot node to start fobot follow task
        # ROS2 param set /follow_copilot follow_detect true
        passfail = self.node_param_util.try_set_node_parameters(node_name, command, type=ParameterType.PARAMETER_BOOL, value=True)
        return passfail


    #
    # self driving task -> JetBot ROS2 laser_avoidance node
    #
    def robot_self_driving(self, goal_handle, self_driving_task):
        self.logger.info("robot_self_driving")

        # self_driving_status = self.command_dict['self-driving']
        self.logger.info("Self-driving status:{}".format(self_driving_task))

        # self driving procedure
        # 1) Ensure the laser_avoidance ROS2 node exist
        # 2) Stop the laser_avoidance node if it is in running state
        # 3) Trun on lader data collecting flag to find out the open area 
        # 4) Roate the robot to face open area
        # 5) Start laser_avoidance node to start self driving task

        # Step 1) Ensure the laser_avoidance ROS2 node exist
        # ros2 param get /laser_avoidance start -- false
        # node_index = self_driving_status.node_index
        # command = self_driving_status.command
        node_name = self_driving_task.node_name
        command = self_driving_task.command
        # passfail, value = self.node_param_util.try_get_node_parameters(node_name, command)
        passfail = self.node_param_util.node_exists(node_name)
        if passfail == False:
            self.mute_ASR_processor(self.node.ASR_node)
            TTS_string = String()
            TTS_string.data = "laser avoidance node not exist, skip the task"
            self.pub_TTS.publish(TTS_string)
            self.logger.info(TTS_string.data)
            return False


        # Step 3) Trun on lader data collecting flag to find out the open area
        self.event.clear()

        # Check for cancellation before starting the task
        if self.node._should_stop or goal_handle.is_cancel_requested:
            self.logger.info("Goal handle is cancel requested, stop the self driving task")
            return False

        uuid_bytes = goal_handle.goal_id.uuid.tobytes()
        uuid_hex = binascii.hexlify(uuid_bytes).decode('utf-8')
        self.logger.info("Goal nandle ID:{} command: {}".format(goal_handle.goal_id, goal_handle.request.command))

        # Step 5) Start laser_avoidance node to start self driving task
        # ROS2 param set /laser_avoidance start true
        passfail = self.node_param_util.try_set_node_parameters(node_name, command, type=ParameterType.PARAMETER_BOOL, value=True)
        return passfail


    #
    # Reset robot long running task by set parameter value to false
    #
    def reset_robot_running_task(self, reset_task):

        node_name = reset_task.node_name
        command = reset_task.command

        self.logger.info("Reset_running task: {} by command:{}".format(node_name, command))


        # Signal the event to unblock any waiting task such as 'follow'
        self.event.set()

        # Check self driving node existance
        passfail = self.node_param_util.node_exists(node_name)

        # If ROS2 node parameter setting not exist skip reset the task
        if passfail == False:
            self.logger.info("{} node not exist, skip the task".format(node_name))
            return passfail

        # Stop the task - ROS2 node if it is in running state
        passfail = self.node_param_util.set_node_parameters(node_name, command, type=ParameterType.PARAMETER_BOOL, value=False)

        return passfail


